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

double smoothStep(double t) {
  const double s = std::min(std::max(t, 0.0), 1.0);
  return s * s * s * (10.0 + s * (-15.0 + s * 6.0));
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

std::array<double, 7> interpolatePosition(const Trajectory& traj, double t) {
  if (t <= 0.0) {
    return traj.q.front();
  }
  if (t >= traj.time.back()) {
    return traj.q.back();
  }
  auto it = std::lower_bound(traj.time.begin(), traj.time.end(), t);
  if (it == traj.time.begin()) {
    return traj.q.front();
  }
  const size_t idx = static_cast<size_t>(std::distance(traj.time.begin(), it));
  const size_t idx0 = idx - 1;
  const double t0 = traj.time[idx0];
  const double t1 = traj.time[idx];
  const double alpha = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;
  std::array<double, 7> q{};
  for (size_t j = 0; j < 7; ++j) {
    q[j] = traj.q[idx0][j] + (traj.q[idx][j] - traj.q[idx0][j]) * alpha;
  }
  return q;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " <robot_ip> <json_path> [time_scale] [start_speed] [max_vel] [max_acc] [vel_alpha] [kp]\n";
    return 1;
  }

  const std::string robot_ip = argv[1];
  const std::string json_path = argv[2];
  const double time_scale = (argc >= 4) ? std::stod(argv[3]) : 1.0;
  const double start_speed = (argc >= 5) ? std::stod(argv[4]) : 0.2;
  const double max_vel = (argc >= 6) ? std::stod(argv[5]) : 2.0;
  const double max_acc = (argc >= 7) ? std::stod(argv[6]) : 5.0;
  const double vel_alpha = (argc >= 8) ? std::stod(argv[7]) : 0.9;
  const double kp = (argc >= 9) ? std::stod(argv[8]) : 2.0;

  try {
    Trajectory traj = loadTrajectory(json_path);

    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);
    robot.automaticErrorRecovery();

    std::cout << "Moving to trajectory start..." << std::endl;
    MotionGenerator move_to_start(start_speed, traj.q.front());
    robot.control(move_to_start);

    const double duration = traj.time.back();
    double time = 0.0;
    std::array<double, 7> last_dq{};
    std::array<double, 7> last_ddq{};
    std::array<double, 7> last_q_des = traj.q.front();
    std::array<double, 7> dq_filtered{};
    auto control_callback = [&](const franka::RobotState& state,
                                franka::Duration period) -> franka::JointVelocities {
      time += period.toSec();
      const double t_traj = time / std::max(1e-6, time_scale);
      if (t_traj >= duration) {
        return franka::MotionFinished(franka::JointVelocities(std::array<double, 7>{}));
      }
      const double ctrl_dt = std::max(1e-6, period.toSec());
      const auto q_des = interpolatePosition(traj, t_traj);
      std::array<double, 7> dq_cmd{};
      for (size_t i = 0; i < 7; ++i) {
        const double dq_raw = (q_des[i] - last_q_des[i]) / ctrl_dt;
        dq_filtered[i] = vel_alpha * dq_filtered[i] + (1.0 - vel_alpha) * dq_raw;
        dq_cmd[i] = dq_filtered[i] + kp * (q_des[i] - state.q_d[i]);
      }
      last_q_des = q_des;

      if (time < 1.0) {
        const double w = smoothStep(time / 1.0);
        for (size_t i = 0; i < 7; ++i) {
          dq_cmd[i] *= w;
        }
      } else if (t_traj > duration - 1.0) {
        const double w = smoothStep((duration - t_traj) / 1.0);
        for (size_t i = 0; i < 7; ++i) {
          dq_cmd[i] *= w;
        }
      }

      const auto upper = franka::computeUpperLimitsJointVelocity(state.q_d);
      const auto lower = franka::computeLowerLimitsJointVelocity(state.q_d);
      const double max_jerk = std::max(1.0, max_acc * 10.0);
      std::array<double, 7> dq_limited{};
      for (size_t i = 0; i < 7; ++i) {
        const double dq_smoothed =
            vel_alpha * last_dq[i] + (1.0 - vel_alpha) * dq_cmd[i];
        dq_limited[i] = franka::limitRate(upper[i],
                                          lower[i],
                                          max_acc,
                                          max_jerk,
                                          dq_smoothed,
                                          last_dq[i],
                                          last_ddq[i]);
      }

      for (size_t i = 0; i < 7; ++i) {
        const double ddq = (dq_limited[i] - last_dq[i]) / ctrl_dt;
        last_ddq[i] = ddq;
        last_dq[i] = dq_limited[i];
      }

      return franka::JointVelocities(dq_limited);
    };

    std::cout << "Tracking trajectory with smoothed velocities (raw, " << duration
              << " s)..." << std::endl;
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
