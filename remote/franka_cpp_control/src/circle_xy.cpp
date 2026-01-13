#include <array>
#include <cmath>
#include <exception>
#include <iostream>
#include <string>

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

double smoothStep(double t) {
  const double s = std::min(std::max(t, 0.0), 1.0);
  return s * s * s * (10.0 + s * (-15.0 + s * 6.0));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <robot_ip> [radius] [angular_speed]\n";
    return 1;
  }

  const std::string robot_ip = argv[1];
  const double radius = (argc >= 3) ? std::stod(argv[2]) : 0.1;
  const double angular_speed = (argc >= 4) ? std::stod(argv[3]) : 0.5;

  const std::array<double, 3> center = {0.3, 0.0, 0.5};
  const double blend_time = 3.0;
  const double total_angle = 4.0 * M_PI;
  const double segment_time = total_angle / angular_speed;

  try {
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);
    robot.automaticErrorRecovery();

    double time = 0.0;
    bool initialized = false;
    std::array<double, 16> initial_pose{};
    std::array<double, 3> start_pos{};
    double start_angle = 0.0;
    auto control_callback = [&](const franka::RobotState& state,
                                franka::Duration period) -> franka::CartesianPose {
      if (!initialized) {
        initial_pose = state.O_T_EE_d;
        start_pos = {initial_pose[12], initial_pose[13], initial_pose[14]};
        const double dx = start_pos[0] - center[0];
        const double dy = start_pos[1] - center[1];
        if (std::hypot(dx, dy) > 1e-6) {
          start_angle = std::atan2(dy, dx);
        }
        initialized = true;
      }

      time += period.toSec();

      double x = start_pos[0];
      double y = start_pos[1];
      double z = start_pos[2];

      if (time < blend_time) {
        const double alpha = smoothStep(time / blend_time);
        const double target_x = center[0] + radius * std::cos(start_angle);
        const double target_y = center[1] + radius * std::sin(start_angle);
        const double target_z = center[2];
        x = start_pos[0] + (target_x - start_pos[0]) * alpha;
        y = start_pos[1] + (target_y - start_pos[1]) * alpha;
        z = start_pos[2] + (target_z - start_pos[2]) * alpha;
      } else {
        const double t_motion = time - blend_time;
        double theta = start_angle;
        if (t_motion < segment_time) {
          const double s = smoothStep(t_motion / segment_time);
          theta = start_angle - total_angle * s;
        } else if (t_motion < 2.0 * segment_time) {
          const double s = smoothStep((t_motion - segment_time) / segment_time);
          theta = start_angle - total_angle + total_angle * s;
        } else {
          std::array<double, 16> finished_pose = initial_pose;
          finished_pose[12] = center[0] + radius * std::cos(start_angle);
          finished_pose[13] = center[1] + radius * std::sin(start_angle);
          finished_pose[14] = center[2];
          return franka::MotionFinished(franka::CartesianPose(finished_pose));
        }

        x = center[0] + radius * std::cos(theta);
        y = center[1] + radius * std::sin(theta);
        z = center[2];
      }

      std::array<double, 16> pose = initial_pose;
      pose[12] = x;
      pose[13] = y;
      pose[14] = z;
      return franka::CartesianPose(pose);
    };

    std::cout << "Drawing circle around (" << center[0] << ", " << center[1]
              << ", " << center[2] << ")..." << std::endl;
    robot.control(control_callback);
    std::cout << "Circle motion finished." << std::endl;
  } catch (const franka::Exception& e) {
    std::cerr << "libfranka exception: " << e.what() << std::endl;
    return 2;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 3;
  }

  return 0;
}
