#include <array>
#include <algorithm>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
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

  bool mapping_ready = false;
  std::array<int, 7> index{};

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

    traj.time.push_back(ts);
    traj.q.push_back(q);
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

Trajectory resample(const Trajectory& input, double hz) {
  const double dt = 1.0 / hz;
  const double duration = input.time.back();

  Trajectory out;
  const size_t count = static_cast<size_t>(duration / dt) + 1;
  out.time.reserve(count);
  out.q.reserve(count);

  size_t idx = 0;
  for (size_t k = 0; k < count; ++k) {
    const double t = k * dt;
    while (idx + 1 < input.time.size() && input.time[idx + 1] < t) {
      idx++;
    }
    const size_t idx_next = std::min(idx + 1, input.time.size() - 1);
    const double t0 = input.time[idx];
    const double t1 = input.time[idx_next];
    const double alpha = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;

    std::array<double, 7> q{};
    for (size_t j = 0; j < 7; ++j) {
      q[j] = input.q[idx][j] + (input.q[idx_next][j] - input.q[idx][j]) * alpha;
    }
    out.time.push_back(t);
    out.q.push_back(q);
  }

  return out;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " <robot_ip> <json_path> [resample_hz] [start_speed]\n";
    return 1;
  }

  const std::string robot_ip = argv[1];
  const std::string json_path = argv[2];
  const double resample_hz = (argc >= 4) ? std::stod(argv[3]) : 200.0;
  const double start_speed = (argc >= 5) ? std::stod(argv[4]) : 0.2;

  try {
    Trajectory raw = loadTrajectory(json_path);
    Trajectory traj = resample(raw, resample_hz);

    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);
    robot.automaticErrorRecovery();

    std::cout << "Moving to trajectory start..." << std::endl;
    MotionGenerator move_to_start(start_speed, traj.q.front());
    robot.control(move_to_start);

    const double duration = traj.time.back();
    double time = 0.0;
    auto control_callback = [&](const franka::RobotState&,
                                franka::Duration period) -> franka::JointPositions {
      time += period.toSec();
      if (time >= duration) {
        return franka::MotionFinished(franka::JointPositions(traj.q.back()));
      }
      const double dt = 1.0 / resample_hz;
      const size_t idx = static_cast<size_t>(time / dt);
      const size_t idx_next = std::min(idx + 1, traj.q.size() - 1);
      const double t0 = idx * dt;
      const double alpha = (time - t0) / dt;

      std::array<double, 7> q_cmd{};
      for (size_t i = 0; i < 7; ++i) {
        q_cmd[i] =
            traj.q[idx][i] + (traj.q[idx_next][i] - traj.q[idx][i]) * alpha;
      }
      return franka::JointPositions(q_cmd);
    };

    std::cout << "Tracking trajectory (raw, " << duration << " s)..."
              << std::endl;
    robot.control(control_callback);
    std::cout << "Trajectory tracking finished." << std::endl;
  } catch (const franka::Exception& e) {
    std::cerr << "libfranka exception: " << e.what() << std::endl;
    return 2;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 3;
  }

  return 0;
}
