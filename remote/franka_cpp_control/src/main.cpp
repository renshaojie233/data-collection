#include <array>
#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/control_types.h>
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

bool getPoseByName(const std::string& name, std::array<double, 7>& q_out) {
  if (name == "ready") {
    q_out = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
    return true;
  }
  if (name == "home") {
    q_out = {0.0, 0.0, 0.0, -1.571, 0.0, 1.571, 0.785};
    return true;
  }
  if (name == "straight") {
    q_out = {0.0, -0.3, 0.0, -2.0, 0.0, 2.0, 0.8};
    return true;
  }
  if (name == "folded") {
    q_out = {0.0, -1.0, 0.0, -2.7, 0.0, 2.3, 0.6};
    return true;
  }
  return false;
}

std::vector<std::string> demoSequence() {
  return {"ready", "straight", "ready", "home", "ready"};
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

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <robot_ip> [speed_factor] [pose|demo]\n";
    return 1;
  }

  const std::string robot_ip = argv[1];
  double speed_factor = 0.2;
  if (argc >= 3) {
    speed_factor = std::stod(argv[2]);
  }

  std::string pose_name = "ready";
  if (argc >= 4) {
    pose_name = argv[3];
  }

  try {
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);
    robot.automaticErrorRecovery();

    if (pose_name == "demo") {
      for (const auto& name : demoSequence()) {
        std::array<double, 7> q{};
        if (!getPoseByName(name, q)) {
          std::cerr << "Unknown pose in demo: " << name << std::endl;
          return 1;
        }
        std::cout << "Moving to pose: " << name << "..." << std::endl;
        MotionGenerator motion(speed_factor, q);
        robot.control(motion);
        std::cout << "Reached pose: " << name << std::endl;
      }
    } else {
      std::array<double, 7> q{};
      if (!getPoseByName(pose_name, q)) {
        std::cerr << "Unknown pose: " << pose_name
                  << " (use ready|home|straight|folded|demo)\n";
        return 1;
      }
      std::cout << "Moving to pose: " << pose_name << "..." << std::endl;
      MotionGenerator motion(speed_factor, q);
      robot.control(motion);
      std::cout << "Reached pose: " << pose_name << std::endl;
    }
  } catch (const franka::Exception& e) {
    std::cerr << "libfranka exception: " << e.what() << std::endl;
    return 2;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 3;
  }

  return 0;
}
