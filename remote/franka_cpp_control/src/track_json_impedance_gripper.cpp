#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <condition_variable>
#include <exception>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

namespace {

void setDefaultBehavior(franka::Robot& robot) {
  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
}

class MotionGenerator {
 public:
  MotionGenerator(double speed_factor, const std::array<double, 7>& q_goal)
      : speed_factor_(speed_factor), q_goal_(q_goal) {}

  franka::JointPositions operator()(const franka::RobotState& state,
                                    franka::Duration period) {
    if (!initialized_) {
      q_start_ = state.q_d;
      double max_delta = 0.0;
      for (size_t i = 0; i < q_goal_.size(); ++i) {
        max_delta = std::max(max_delta, std::abs(q_goal_[i] - q_start_[i]));
      }
      const double max_speed = std::max(1e-6, 2.0 * speed_factor_);
      time_to_goal_ = std::max(0.5, max_delta / max_speed);
      initialized_ = true;
    }

    time_ += period.toSec();
    const double t = std::min(time_ / time_to_goal_, 1.0);
    const double alpha = t * t * t * (10.0 + t * (-15.0 + t * 6.0));
    std::array<double, 7> q_d{};
    for (size_t i = 0; i < q_goal_.size(); ++i) {
      q_d[i] = q_start_[i] + (q_goal_[i] - q_start_[i]) * alpha;
    }

    if (alpha >= 1.0) {
      return franka::MotionFinished(franka::JointPositions(q_goal_));
    }
    return franka::JointPositions(q_d);
  }

 private:
  double speed_factor_{0.2};
  std::array<double, 7> q_goal_{};
  std::array<double, 7> q_start_{};
  bool initialized_{false};
  double time_{0.0};
  double time_to_goal_{1.0};
};

struct Trajectory {
  std::vector<double> time;
  std::vector<std::array<double, 7>> q;
  std::vector<double> gripper_width;
};

Trajectory loadTrajectory(const std::string& path) {
  namespace pt = boost::property_tree;
  pt::ptree root;
  pt::read_json(path, root);

  const std::array<std::string, 7> canonical = {
      "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
      "fr3_joint5", "fr3_joint6", "fr3_joint7"};

  Trajectory traj;
  traj.time.reserve(10000);
  traj.q.reserve(10000);
  traj.gripper_width.reserve(10000);

  bool mapping_ready = false;
  bool gripper_mapping_ready = false;
  std::array<int, 7> index{};
  int finger_1_index = -1;
  int finger_2_index = -1;

  for (const auto& entry : root.get_child("data")) {
    const auto& node = entry.second;
    const double ts = node.get<double>("timestamp");

    const auto& franka = node.get_child("franka_joints");
    if (!mapping_ready) {
      std::vector<std::string> names;
      for (const auto& name : franka.get_child("names")) {
        names.push_back(name.second.get_value<std::string>());
      }
      for (size_t i = 0; i < canonical.size(); ++i) {
        const auto it = std::find(names.begin(), names.end(), canonical[i]);
        if (it == names.end()) {
          throw std::runtime_error("Missing joint name in franka_joints.names");
        }
        index[i] = static_cast<int>(std::distance(names.begin(), it));
      }
      mapping_ready = true;
    }

    std::vector<double> pos;
    for (const auto& v : franka.get_child("position")) {
      pos.push_back(v.second.get_value<double>());
    }
    if (pos.size() < 7) {
      throw std::runtime_error("Invalid franka_joints.position length");
    }
    std::array<double, 7> q{};
    for (size_t i = 0; i < 7; ++i) {
      q[i] = pos[static_cast<size_t>(index[i])];
    }

    const auto& gripper = node.get_child("gripper_joints");
    if (!gripper_mapping_ready) {
      std::vector<std::string> g_names;
      for (const auto& name : gripper.get_child("names")) {
        g_names.push_back(name.second.get_value<std::string>());
      }
      const auto it1 = std::find(g_names.begin(), g_names.end(), "fr3_finger_joint1");
      const auto it2 = std::find(g_names.begin(), g_names.end(), "fr3_finger_joint2");
      if (it1 == g_names.end() || it2 == g_names.end()) {
        throw std::runtime_error("Missing finger joint names in gripper_joints.names");
      }
      finger_1_index = static_cast<int>(std::distance(g_names.begin(), it1));
      finger_2_index = static_cast<int>(std::distance(g_names.begin(), it2));
      gripper_mapping_ready = true;
    }

    std::vector<double> g_pos;
    for (const auto& v : gripper.get_child("position")) {
      g_pos.push_back(v.second.get_value<double>());
    }
    const size_t g_size = g_pos.size();
    if (finger_1_index < 0 || finger_2_index < 0 ||
        g_size <= static_cast<size_t>(finger_1_index) ||
        g_size <= static_cast<size_t>(finger_2_index)) {
      throw std::runtime_error("Invalid gripper_joints.position length");
    }
    const double width =
        g_pos[static_cast<size_t>(finger_1_index)] + g_pos[static_cast<size_t>(finger_2_index)];

    traj.time.push_back(ts);
    traj.q.push_back(q);
    traj.gripper_width.push_back(width);
  }

  if (traj.time.size() < 2) {
    throw std::runtime_error("Not enough trajectory points in JSON");
  }

  const double t0 = traj.time.front();
  for (auto& t : traj.time) {
    t -= t0;
  }

  return traj;
}

void interpolate(const Trajectory& traj,
                 double t,
                 std::array<double, 7>& q_out,
                 std::array<double, 7>& dq_out) {
  if (t <= 0.0) {
    q_out = traj.q.front();
    dq_out = {};
    return;
  }
  if (t >= traj.time.back()) {
    q_out = traj.q.back();
    dq_out = {};
    return;
  }
  auto it = std::lower_bound(traj.time.begin(), traj.time.end(), t);
  if (it == traj.time.begin()) {
    q_out = traj.q.front();
    dq_out = {};
    return;
  }
  const size_t idx1 = static_cast<size_t>(std::distance(traj.time.begin(), it));
  const size_t idx0 = idx1 - 1;
  const double t0 = traj.time[idx0];
  const double t1 = traj.time[idx1];
  const double alpha = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;

  for (size_t i = 0; i < 7; ++i) {
    q_out[i] = traj.q[idx0][i] + (traj.q[idx1][i] - traj.q[idx0][i]) * alpha;
    dq_out[i] = (traj.q[idx1][i] - traj.q[idx0][i]) / std::max(1e-6, t1 - t0);
  }
}

std::vector<bool> buildDesiredOpen(const Trajectory& traj, double open_threshold) {
  const size_t n = traj.time.size();
  std::vector<bool> desired_open(n, false);
  const double eps = 1e-4;

  for (size_t i = 0; i < n; ++i) {
    if (traj.gripper_width[i] >= open_threshold) {
      desired_open[i] = true;
    }
  }

  size_t i = 0;
  while (i + 1 < n) {
    if (traj.gripper_width[i + 1] <= traj.gripper_width[i] + eps) {
      ++i;
      continue;
    }
    const size_t start = i;
    size_t j = i + 1;
    size_t reach = n;
    if (traj.gripper_width[j] >= open_threshold) {
      reach = j;
    }
    while (j + 1 < n && traj.gripper_width[j + 1] >= traj.gripper_width[j] - eps) {
      ++j;
      if (reach == n && traj.gripper_width[j] >= open_threshold) {
        reach = j;
      }
    }
    if (reach < n && traj.gripper_width[start] < open_threshold) {
      for (size_t k = start; k <= reach; ++k) {
        desired_open[k] = true;
      }
    }
    i = j;
  }

  return desired_open;
}

bool desiredOpenAtTime(const Trajectory& traj,
                       const std::vector<bool>& desired_open,
                       double t) {
  if (t <= 0.0) {
    return desired_open.front();
  }
  if (t >= traj.time.back()) {
    return desired_open.back();
  }
  auto it = std::lower_bound(traj.time.begin(), traj.time.end(), t);
  if (it == traj.time.begin()) {
    return desired_open.front();
  }
  const size_t idx1 = static_cast<size_t>(std::distance(traj.time.begin(), it));
  const size_t idx0 = idx1 - 1;
  return desired_open[idx0];
}

// Gripper control thread function
void gripperControlThread(const std::string& robot_ip,
                          const Trajectory& traj,
                          const std::vector<bool>& desired_open,
                          double time_scale,
                          double gripper_speed,
                          double gripper_force,
                          std::atomic<double>& shared_time,
                          std::atomic<bool>& arm_control_started,
                          std::atomic<bool>& arm_control_finished,
                          std::atomic<bool>& gripper_should_stop,
                          std::exception_ptr& gripper_error,
                          std::mutex& error_mutex) {
  try {
    franka::Gripper gripper(robot_ip);

    // Wait for arm control to start
    std::cout << "[Gripper] Waiting for arm to reach starting position..." << std::endl;
    while (!arm_control_started.load() && !gripper_should_stop.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (gripper_should_stop.load()) {
      return;
    }

    std::cout << "[Gripper] Homing gripper..." << std::endl;
    if (!gripper.homing()) {
      throw std::runtime_error("Gripper homing failed");
    }
    const double max_width = gripper.readOnce().max_width;
    const double open_threshold = max_width - 1e-4;

    const double duration = traj.time.back();
    const auto min_command_interval = std::chrono::milliseconds(200);
    auto last_cmd_time = std::chrono::steady_clock::now() - min_command_interval;
    bool last_open = false;
    bool have_state = false;
    std::atomic<bool> gripper_busy{false};
    std::thread cmd_thread;

    auto start_command = [&](bool open) {
      if (cmd_thread.joinable()) {
        cmd_thread.join();
      }
      gripper_busy.store(true);
      cmd_thread = std::thread([&, open]() {
        try {
          if (open) {
            gripper.move(max_width, gripper_speed);
          } else {
            gripper.grasp(0.0, gripper_speed, gripper_force);
          }
        } catch (...) {
          std::lock_guard<std::mutex> lock(error_mutex);
          if (!gripper_error) {
            gripper_error = std::current_exception();
          }
        }
        gripper_busy.store(false);
      });
    };

    std::cout << "[Gripper] Starting trajectory tracking..." << std::endl;

    while (!arm_control_finished.load() && !gripper_should_stop.load()) {
      const double t_elapsed = shared_time.load();
      const double t_traj = t_elapsed / std::max(1e-6, time_scale);
      const double t_clamped = std::min(t_traj, duration);
      const bool is_open = desiredOpenAtTime(traj, desired_open, t_clamped);

      if (!have_state || is_open != last_open) {
        const auto now = std::chrono::steady_clock::now();
        if (!gripper_busy.load() && now - last_cmd_time >= min_command_interval) {
          start_command(is_open);
          last_cmd_time = now;
          last_open = is_open;
          have_state = true;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (cmd_thread.joinable()) {
      cmd_thread.join();
    }

    std::cout << "[Gripper] Control finished." << std::endl;
  } catch (...) {
    std::lock_guard<std::mutex> lock(error_mutex);
    if (!gripper_error) {
      gripper_error = std::current_exception();
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " <robot_ip> <json_path> [time_scale] [start_speed] [k] [d] [k_alpha] [q_alpha]"
                 " [gripper_speed] [gripper_force]\n";
    return 1;
  }

  const std::string robot_ip = argv[1];
  const std::string json_path = argv[2];
  const double time_scale = (argc >= 4) ? std::stod(argv[3]) : 1.0;
  const double start_speed = (argc >= 5) ? std::stod(argv[4]) : 0.2;
  const double k_gain = (argc >= 6) ? std::stod(argv[5]) : 30.0;
  const double d_gain = (argc >= 7) ? std::stod(argv[6]) : 5.0;
  const double k_alpha = (argc >= 8) ? std::stod(argv[7]) : 0.3;
  const double q_alpha = (argc >= 9) ? std::stod(argv[8]) : 0.2;
  const double gripper_speed = (argc >= 10) ? std::stod(argv[9]) : 0.1;
  const double gripper_force = (argc >= 11) ? std::stod(argv[10]) : 20.0;

  std::array<double, 7> k_gains{};
  std::array<double, 7> d_gains{};
  k_gains.fill(k_gain);
  d_gains.fill(d_gain);

  if (k_alpha < 0.0 || k_alpha > 1.0 || q_alpha < 0.0 || q_alpha > 1.0) {
    std::cerr << "k_alpha and q_alpha should be in [0, 1]\n";
    return 1;
  }

  try {
    Trajectory traj = loadTrajectory(json_path);

    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);
    robot.automaticErrorRecovery();

    // Prepare gripper desired state
    franka::Gripper gripper_temp(robot_ip);
    const double max_width = gripper_temp.readOnce().max_width;
    const double open_threshold = max_width - 1e-4;
    const auto desired_open = buildDesiredOpen(traj, open_threshold);

    // Shared state for synchronization
    std::atomic<double> shared_time{0.0};
    std::atomic<bool> arm_control_started{false};
    std::atomic<bool> arm_control_finished{false};
    std::atomic<bool> gripper_should_stop{false};
    std::exception_ptr gripper_error = nullptr;
    std::mutex error_mutex;

    // Start gripper control thread
    std::thread gripper_thread(gripperControlThread,
                               std::cref(robot_ip),
                               std::cref(traj),
                               std::cref(desired_open),
                               time_scale,
                               gripper_speed,
                               gripper_force,
                               std::ref(shared_time),
                               std::ref(arm_control_started),
                               std::ref(arm_control_finished),
                               std::ref(gripper_should_stop),
                               std::ref(gripper_error),
                               std::ref(error_mutex));

    std::cout << "[Arm] Moving to trajectory start..." << std::endl;
    MotionGenerator move_to_start(start_speed, traj.q.front());
    robot.control(move_to_start);

    // Signal that arm is ready
    arm_control_started.store(true);
    std::cout << "[Arm] Reached starting position, signaling gripper..." << std::endl;

    auto model = robot.loadModel();

    const double duration = traj.time.back();
    double time = 0.0;
    double settle_time = 0.0;
    const double stop_threshold = 0.02;
    const double settle_required = 0.5;
    bool initialized = false;
    std::array<double, 7> dq_filtered{};
    std::array<double, 7> last_tau{};
    std::array<double, 7> q_goal_filtered{};
    std::array<double, 7> q_goal_prev{};

    auto control_callback = [&](const franka::RobotState& state,
                                franka::Duration period) -> franka::Torques {
      if (!initialized) {
        dq_filtered = {};
        last_tau = model.coriolis(state);
        q_goal_filtered = traj.q.front();
        q_goal_prev = traj.q.front();
        initialized = true;
      }

      time += period.toSec();
      shared_time.store(time);  // Update shared time for gripper thread

      const double t_traj = time / std::max(1e-6, time_scale);
      const double t_clamped = std::min(t_traj, duration);

      std::array<double, 7> q_goal{};
      std::array<double, 7> dq_goal{};
      interpolate(traj, t_clamped, q_goal, dq_goal);
      if (t_traj >= duration) {
        dq_goal = {};
      }
      for (size_t i = 0; i < 7; ++i) {
        q_goal_filtered[i] =
            (1.0 - q_alpha) * q_goal_filtered[i] + q_alpha * q_goal[i];
      }
      const double dt = std::max(1e-6, period.toSec());
      for (size_t i = 0; i < 7; ++i) {
        dq_goal[i] = (q_goal_filtered[i] - q_goal_prev[i]) / dt;
      }
      q_goal_prev = q_goal_filtered;

      for (size_t i = 0; i < 7; ++i) {
        dq_filtered[i] = (1.0 - k_alpha) * dq_filtered[i] + k_alpha * state.dq[i];
      }

      const auto coriolis = model.coriolis(state);
      std::array<double, 7> tau_d{};
      for (size_t i = 0; i < 7; ++i) {
        tau_d[i] = coriolis[i] + k_gains[i] * (q_goal_filtered[i] - state.q[i]) +
                   d_gains[i] * (dq_goal[i] - dq_filtered[i]);
      }

      const auto tau_limited = franka::limitRate(franka::kMaxTorqueRate, tau_d, last_tau);
      last_tau = tau_limited;
      if (t_traj >= duration) {
        double max_dq = 0.0;
        for (double v : state.dq) {
          max_dq = std::max(max_dq, std::abs(v));
        }
        if (max_dq < stop_threshold) {
          settle_time += period.toSec();
        } else {
          settle_time = 0.0;
        }
        if (settle_time >= settle_required) {
          return franka::MotionFinished(franka::Torques(tau_limited));
        }
      }
      return franka::Torques(tau_limited);
    };

    std::cout << "[Arm] Tracking trajectory with joint impedance (filtered, " << duration << " s)..."
              << std::endl;
    robot.control(control_callback);
    std::cout << "[Arm] Trajectory tracking finished." << std::endl;

    // Signal gripper thread to finish
    arm_control_finished.store(true);
    gripper_thread.join();

    // Check for gripper errors
    {
      std::lock_guard<std::mutex> lock(error_mutex);
      if (gripper_error) {
        std::rethrow_exception(gripper_error);
      }
    }

    std::cout << "All controls finished successfully." << std::endl;
  } catch (const franka::Exception& e) {
    std::cerr << "libfranka exception: " << e.what() << std::endl;
    return 2;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 3;
  }

  return 0;
}
