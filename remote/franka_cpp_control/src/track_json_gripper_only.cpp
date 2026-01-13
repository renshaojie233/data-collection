#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <exception>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <franka/exception.h>
#include <franka/gripper.h>

namespace {

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

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " <robot_ip> <json_path> [time_scale] [start_speed] [gripper_speed]"
                 " [start_delay]\n";
    return 1;
  }

  const std::string robot_ip = argv[1];
  const std::string json_path = argv[2];
  const double time_scale = (argc >= 4) ? std::stod(argv[3]) : 1.0;
  const double start_speed = (argc >= 5) ? std::stod(argv[4]) : 0.2;
  const double gripper_speed = (argc >= 6) ? std::stod(argv[5]) : 0.1;
  const double start_delay = (argc >= 7) ? std::stod(argv[6]) : 0.5;

  try {
    Trajectory traj = loadTrajectory(json_path);

    franka::Gripper gripper(robot_ip);

    std::cout << "Waiting " << start_delay << " s for arm alignment..." << std::endl;
    if (start_delay > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(start_delay));
    }

    std::cout << "Homing gripper after alignment..." << std::endl;
    if (!gripper.homing()) {
      throw std::runtime_error("Gripper homing failed");
    }
    const double max_width = gripper.readOnce().max_width;
    const double open_threshold = max_width - 1e-4;
    const auto desired_open = buildDesiredOpen(traj, open_threshold);

    const double duration = traj.time.back();
    const auto min_command_interval = std::chrono::milliseconds(200);
    const auto start = std::chrono::steady_clock::now();
    auto last_cmd_time = start - min_command_interval;
    bool last_open = false;
    bool have_state = false;
    std::atomic<bool> gripper_busy{false};
    std::exception_ptr gripper_error = nullptr;
    std::mutex gripper_error_mutex;
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
            gripper.move(0.0, gripper_speed);
          }
        } catch (...) {
          std::lock_guard<std::mutex> lock(gripper_error_mutex);
          if (!gripper_error) {
            gripper_error = std::current_exception();
          }
        }
        gripper_busy.store(false);
      });
    };

    while (true) {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed =
          std::chrono::duration<double>(now - start).count();
      const double t_traj = elapsed / std::max(1e-6, time_scale);
      const double t_clamped = std::min(t_traj, duration);
      const bool is_open = desiredOpenAtTime(traj, desired_open, t_clamped);

      if (!have_state || is_open != last_open) {
        if (!gripper_busy.load() &&
            now - last_cmd_time >= min_command_interval) {
          start_command(is_open);
          last_cmd_time = now;
          last_open = is_open;
          have_state = true;
        }
      }

      if (t_traj >= duration) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (cmd_thread.joinable()) {
      cmd_thread.join();
    }
    if (gripper_error) {
      std::rethrow_exception(gripper_error);
    }
    (void)start_speed;
  } catch (const franka::Exception& e) {
    std::cerr << "libfranka exception: " << e.what() << std::endl;
    return 2;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 3;
  }

  return 0;
}
