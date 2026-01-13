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

std::array<double, 7> loadStartPosition(const std::string& path) {
  namespace pt = boost::property_tree;
  pt::ptree root;
  pt::read_json(path, root);

  const std::array<std::string, 7> canonical = {
      "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
      "fr3_joint5", "fr3_joint6", "fr3_joint7"};

  const auto& data = root.get_child("data");
  if (data.empty()) {
    throw std::runtime_error("JSON data is empty");
  }
  const auto& node = data.front().second;
  const auto& franka = node.get_child("franka_joints");

  std::vector<std::string> names;
  for (const auto& name : franka.get_child("names")) {
    names.push_back(name.second.get_value<std::string>());
  }

  std::vector<double> pos;
  for (const auto& v : franka.get_child("position")) {
    pos.push_back(v.second.get_value<double>());
  }
  if (pos.size() < 7 || names.size() < 7) {
    throw std::runtime_error("Invalid franka_joints fields");
  }

  std::array<double, 7> q{};
  for (size_t i = 0; i < canonical.size(); ++i) {
    const auto it = std::find(names.begin(), names.end(), canonical[i]);
    if (it == names.end()) {
      throw std::runtime_error("Missing joint name in franka_joints.names");
    }
    const size_t idx = static_cast<size_t>(std::distance(names.begin(), it));
    q[i] = pos[idx];
  }
  return q;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <robot_ip> <json_path> [start_speed]\n";
    return 1;
  }

  const std::string robot_ip = argv[1];
  const std::string json_path = argv[2];
  const double start_speed = (argc >= 4) ? std::stod(argv[3]) : 0.1;

  try {
    const auto q_goal = loadStartPosition(json_path);
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);
    robot.automaticErrorRecovery();
    MotionGenerator move_to_start(start_speed, q_goal);
    robot.control(move_to_start);
  } catch (const franka::Exception& e) {
    std::cerr << "libfranka exception: " << e.what() << std::endl;
    return 2;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 3;
  }

  return 0;
}
