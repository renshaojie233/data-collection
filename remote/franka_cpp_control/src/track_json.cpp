#include <array>
#include <algorithm>
#include <cmath>
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

struct Trajectory {
  std::vector<double> time;
  std::vector<std::array<double, 7>> q;
};

struct Spline1D {
  std::vector<double> y;
  std::vector<double> y2;
  double dt{0.0};
};

class MotionGenerator {
 public:
  MotionGenerator(double speed_factor, const std::array<double, 7>& q_goal)
      : speed_factor_(speed_factor), q_goal_(q_goal) {}

  double duration(const std::array<double, 7>& q_start) const {
    double max_delta = 0.0;
    for (size_t i = 0; i < q_goal_.size(); ++i) {
      max_delta = std::max(max_delta, std::abs(q_goal_[i] - q_start[i]));
    }
    const double max_speed = std::max(1e-6, 2.0 * speed_factor_);
    return std::max(0.5, max_delta / max_speed);
  }

  std::array<double, 7> position(const std::array<double, 7>& q_start,
                                 double t,
                                 double total_time) const {
    const double tau = std::min(t / total_time, 1.0);
    const double alpha = tau * tau * tau * (10.0 + tau * (-15.0 + tau * 6.0));
    std::array<double, 7> q_d{};
    for (size_t i = 0; i < q_goal_.size(); ++i) {
      q_d[i] = q_start[i] + (q_goal_[i] - q_start[i]) * alpha;
    }
    return q_d;
  }

 private:
  double speed_factor_{0.2};
  std::array<double, 7> q_goal_{};
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

Trajectory smoothTrajectory(const Trajectory& input, int window) {
  if (window <= 1) {
    return input;
  }
  Trajectory out = input;
  const int half = window / 2;
  for (size_t i = 0; i < input.q.size(); ++i) {
    std::array<double, 7> acc{};
    int count = 0;
    const int start = static_cast<int>(i) - half;
    const int end = static_cast<int>(i) + half;
    for (int k = start; k <= end; ++k) {
      if (k < 0 || k >= static_cast<int>(input.q.size())) {
        continue;
      }
      for (size_t j = 0; j < 7; ++j) {
        acc[j] += input.q[static_cast<size_t>(k)][j];
      }
      count++;
    }
    for (size_t j = 0; j < 7; ++j) {
      out.q[i][j] = acc[j] / static_cast<double>(count);
    }
  }
  return out;
}

double maxJointVelocity(const Trajectory& traj, double dt) {
  double max_v = 0.0;
  for (size_t i = 1; i < traj.q.size(); ++i) {
    for (size_t j = 0; j < 7; ++j) {
      const double v = std::abs((traj.q[i][j] - traj.q[i - 1][j]) / dt);
      if (v > max_v) {
        max_v = v;
      }
    }
  }
  return max_v;
}

Spline1D buildSpline(const std::vector<double>& y, double dt) {
  Spline1D spline;
  spline.y = y;
  spline.y2.assign(y.size(), 0.0);
  spline.dt = dt;

  const size_t n = y.size();
  if (n < 2) {
    throw std::runtime_error("Spline requires at least 2 points");
  }

  std::vector<double> u(n - 1, 0.0);
  spline.y2[0] = 0.0;
  u[0] = 0.0;

  for (size_t i = 1; i + 1 < n; ++i) {
    const double sig = 0.5;
    const double p = sig * spline.y2[i - 1] + 2.0;
    spline.y2[i] = (sig - 1.0) / p;
    const double dd = (y[i + 1] - 2.0 * y[i] + y[i - 1]) / (dt * dt);
    u[i] = (6.0 * dd - sig * u[i - 1]) / p;
  }

  spline.y2[n - 1] = 0.0;
  for (size_t k = n - 2; k < n; --k) {
    spline.y2[k] = spline.y2[k] * spline.y2[k + 1] + u[k];
    if (k == 0) {
      break;
    }
  }

  return spline;
}

double evalSpline(const Spline1D& s, double t) {
  const size_t n = s.y.size();
  if (t <= 0.0) {
    return s.y.front();
  }
  const double tmax = (n - 1) * s.dt;
  if (t >= tmax) {
    return s.y.back();
  }
  const size_t i = static_cast<size_t>(t / s.dt);
  const double t0 = i * s.dt;
  const double a = (t0 + s.dt - t) / s.dt;
  const double b = (t - t0) / s.dt;
  return a * s.y[i] + b * s.y[i + 1] +
         ((a * a * a - a) * s.y2[i] + (b * b * b - b) * s.y2[i + 1]) *
             (s.dt * s.dt) / 6.0;
}

double maxDerivative(const std::array<Spline1D, 7>& splines,
                     double duration,
                     double dt,
                     int order) {
  double max_val = 0.0;
  std::array<double, 7> prev{};
  std::array<double, 7> prev2{};
  bool has_prev = false;
  bool has_prev2 = false;

  for (double t = 0.0; t <= duration; t += dt) {
    std::array<double, 7> q{};
    for (size_t j = 0; j < 7; ++j) {
      q[j] = evalSpline(splines[j], t);
    }
    if (order == 1 && has_prev) {
      for (size_t j = 0; j < 7; ++j) {
        max_val = std::max(max_val, std::abs((q[j] - prev[j]) / dt));
      }
    } else if (order == 2 && has_prev2) {
      for (size_t j = 0; j < 7; ++j) {
        const double v1 = (prev[j] - prev2[j]) / dt;
        const double v2 = (q[j] - prev[j]) / dt;
        max_val = std::max(max_val, std::abs((v2 - v1) / dt));
      }
    }
    prev2 = prev;
    prev = q;
    has_prev2 = has_prev;
    has_prev = true;
  }

  return max_val;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " <robot_ip> <json_path> [resample_hz] [start_speed] [max_vel] [max_acc] [smooth_window]\n";
    return 1;
  }

  const std::string robot_ip = argv[1];
  const std::string json_path = argv[2];
  const double resample_hz = (argc >= 4) ? std::stod(argv[3]) : 200.0;
  const double start_speed = (argc >= 5) ? std::stod(argv[4]) : 0.2;
  const double max_vel = (argc >= 6) ? std::stod(argv[5]) : 1.0;
  const double max_acc = (argc >= 7) ? std::stod(argv[6]) : 2.0;
  const int smooth_window = (argc >= 8) ? std::stoi(argv[7]) : 11;

  try {
    Trajectory raw = loadTrajectory(json_path);
    Trajectory resampled = resample(raw, resample_hz);
    Trajectory traj = smoothTrajectory(resampled, smooth_window);

    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);
    robot.automaticErrorRecovery();

    const double duration = traj.time.back();
    const double sample_dt = 1.0 / resample_hz;
    std::array<Spline1D, 7> splines;
    for (size_t j = 0; j < 7; ++j) {
      std::vector<double> y(traj.q.size());
      for (size_t i = 0; i < traj.q.size(); ++i) {
        y[i] = traj.q[i][j];
      }
      splines[j] = buildSpline(y, sample_dt);
    }

    const double observed_max_v = maxDerivative(splines, duration, sample_dt, 1);
    const double observed_max_a = maxDerivative(splines, duration, sample_dt, 2);
    const double scale_v = (max_vel > 0.0) ? observed_max_v / max_vel : 1.0;
    const double scale_a =
        (max_acc > 0.0) ? std::sqrt(observed_max_a / max_acc) : 1.0;
    const double time_scale = std::max(1.0, std::max(scale_v, scale_a));
    const double total_time = duration * time_scale;
    std::cout << "Observed max vel " << observed_max_v << " rad/s, max acc "
              << observed_max_a << " rad/s^2" << std::endl;
    std::cout << "Planned duration " << total_time << " s" << std::endl;

    MotionGenerator move_to_start(start_speed, traj.q.front());
    bool initialized = false;
    std::array<double, 7> q_init{};
    double start_duration = 0.0;
    std::array<double, 7> last_q{};
    std::array<double, 7> last_dq{};
    std::array<double, 7> last_ddq{};
    double time = 0.0;

    auto control_callback = [&](const franka::RobotState& state,
                                franka::Duration period) -> franka::JointPositions {
      if (!initialized) {
        q_init = state.q_d;
        start_duration = move_to_start.duration(q_init);
        last_q = state.q_d;
        last_dq = {};
        last_ddq = {};
        initialized = true;
      }

      time += period.toSec();

      if (time < start_duration) {
        const auto q_cmd = move_to_start.position(q_init, time, start_duration);
        return franka::JointPositions(q_cmd);
      }

      const double t_traj = time - start_duration;
      if (total_time <= 0.0) {
        return franka::MotionFinished(franka::JointPositions(traj.q.back()));
      }
      const double s = smoothStep(std::min(t_traj / total_time, 1.0));
      const double t_warp = s * duration;
      if (t_warp >= duration) {
        return franka::MotionFinished(franka::JointPositions(traj.q.back()));
      }

      const double sample_dt = 1.0 / resample_hz;
      const size_t idx = static_cast<size_t>(t_warp / sample_dt);
      const size_t idx_next = std::min(idx + 1, traj.q.size() - 1);
      const double t0 = idx * sample_dt;
      const double alpha = (t_warp - t0) / sample_dt;

      std::array<double, 7> q_traj{};
      for (size_t i = 0; i < 7; ++i) {
        q_traj[i] = evalSpline(splines[i], t_warp);
      }

      const auto upper = franka::computeUpperLimitsJointVelocity(last_q);
      const auto lower = franka::computeLowerLimitsJointVelocity(last_q);
      const double max_jerk = std::max(1.0, max_acc * 10.0);
      std::array<double, 7> q_cmd{};
      for (size_t i = 0; i < 7; ++i) {
        q_cmd[i] = franka::limitRate(upper[i],
                                     lower[i],
                                     max_acc,
                                     max_jerk,
                                     q_traj[i],
                                     last_q[i],
                                     last_dq[i],
                                     last_ddq[i]);
      }
      const double dt = std::max(1e-6, period.toSec());
      for (size_t i = 0; i < 7; ++i) {
        const double dq = (q_cmd[i] - last_q[i]) / dt;
        const double ddq = (dq - last_dq[i]) / dt;
        last_q[i] = q_cmd[i];
        last_dq[i] = dq;
        last_ddq[i] = ddq;
      }
      return franka::JointPositions(q_cmd);
    };

    std::cout << "Moving to trajectory start and tracking (start "
              << start_speed << ", duration " << duration << " s, scale "
              << time_scale << ")..." << std::endl;
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
