#include <array>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace {

struct RobotiqConfig {
  std::string port{"/dev/ttyUSB0"};
  int baud{115200};
  int slave_id{9};
  int out_start{0x03E8};
  int in_start{0x07D0};
  int timeout_ms{200};
  int min_command_interval_ms{200};
  int speed{128};
  int force{128};
  double source_max_width{0.08};
  double position_eps{0.02};
  bool codex_layout{true};
  bool hold_initial{false};
  int activation_mode{0};
};

double clampDouble(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

int clampInt(int value, int low, int high) {
  return std::max(low, std::min(high, value));
}

int toRobotiqByte(double value, int fallback) {
  if (std::isnan(value) || std::isinf(value)) {
    return fallback;
  }
  if (value <= 1.0) {
    return clampInt(static_cast<int>(std::lround(value * 255.0)), 0, 255);
  }
  return clampInt(static_cast<int>(std::lround(value)), 0, 255);
}

std::string envString(const char* name, const std::string& fallback) {
  const char* value = std::getenv(name);
  if (!value || !value[0]) {
    return fallback;
  }
  return std::string(value);
}

std::string toLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

int parseActivationMode(const std::string& value) {
  const std::string lowered = toLower(value);
  if (lowered == "skip" || lowered == "0" || lowered == "false") {
    return 2;
  }
  if (lowered == "force" || lowered == "1" || lowered == "true") {
    return 1;
  }
  return 0;
}

int envInt(const char* name, int fallback) {
  const char* value = std::getenv(name);
  if (!value || !value[0]) {
    return fallback;
  }
  try {
    size_t idx = 0;
    int parsed = std::stoi(value, &idx, 0);
    if (idx == 0) {
      return fallback;
    }
    return parsed;
  } catch (...) {
    return fallback;
  }
}

double envDouble(const char* name, double fallback) {
  const char* value = std::getenv(name);
  if (!value || !value[0]) {
    return fallback;
  }
  try {
    size_t idx = 0;
    double parsed = std::stod(value, &idx);
    if (idx == 0) {
      return fallback;
    }
    return parsed;
  } catch (...) {
    return fallback;
  }
}

int envRobotiqByte(const char* name, int fallback) {
  const char* value = std::getenv(name);
  if (!value || !value[0]) {
    return fallback;
  }
  try {
    size_t idx = 0;
    double parsed = std::stod(value, &idx);
    if (idx == 0) {
      return fallback;
    }
    return toRobotiqByte(parsed, fallback);
  } catch (...) {
    return fallback;
  }
}

uint16_t crc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]);
    for (int j = 0; j < 8; ++j) {
      if (crc & 1) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

speed_t baudToSpeed(int baud) {
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:
      return B115200;
  }
}

class SerialPort {
 public:
  SerialPort() = default;
  ~SerialPort() { close(); }

  void open(const std::string& port, int baud) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      throw std::runtime_error("Failed to open " + port + ": " + std::strerror(errno));
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      const std::string err = std::strerror(errno);
      close();
      throw std::runtime_error("tcgetattr failed: " + err);
    }

    speed_t speed = baudToSpeed(baud);
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      const std::string err = std::strerror(errno);
      close();
      throw std::runtime_error("tcsetattr failed: " + err);
    }

    tcflush(fd_, TCIOFLUSH);
  }

  bool isOpen() const { return fd_ >= 0; }

  void close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool writeAll(const uint8_t* data, size_t length, std::string& error) {
    size_t written = 0;
    while (written < length) {
      ssize_t out = ::write(fd_, data + written, length - written);
      if (out < 0) {
        error = std::strerror(errno);
        return false;
      }
      written += static_cast<size_t>(out);
    }
    tcdrain(fd_);
    return true;
  }

  bool readExact(size_t length,
                 std::chrono::milliseconds timeout,
                 std::vector<uint8_t>& out,
                 std::string& error) {
    out.clear();
    out.reserve(length);
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    while (out.size() < length) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        break;
      }
      const auto remaining =
          std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
      pollfd pfd{};
      pfd.fd = fd_;
      pfd.events = POLLIN;
      const int rc = ::poll(&pfd, 1, static_cast<int>(remaining.count()));
      if (rc < 0) {
        error = std::strerror(errno);
        return false;
      }
      if (rc == 0) {
        continue;
      }
      if (pfd.revents & POLLIN) {
        uint8_t buffer[64];
        const size_t remaining_len = length - out.size();
        const size_t to_read = std::min(sizeof(buffer), remaining_len);
        const ssize_t got = ::read(fd_, buffer, to_read);
        if (got < 0) {
          error = std::strerror(errno);
          return false;
        }
        if (got > 0) {
          out.insert(out.end(), buffer, buffer + got);
        }
      }
    }
    return out.size() == length;
  }

 private:
  int fd_{-1};
};

class RobotiqModbus {
 public:
  explicit RobotiqModbus(RobotiqConfig config) : config_(std::move(config)) {}

  void connect() { serial_.open(config_.port, config_.baud); }

  void close() { serial_.close(); }

  bool activate(std::string& error) {
    if (config_.activation_mode == 2) {
      return true;
    }
    if (config_.activation_mode == 0) {
      RobotiqStatus status{};
      std::string status_error;
      if (readStatus(status, status_error)) {
        if (status.gACT == 1 && status.gSTA >= 2) {
          return true;
        }
      }
    }
    const int speed = clampInt(config_.speed, 0, 255);
    const int force = clampInt(config_.force, 0, 255);
    if (!writeCommand(0, 0, 0, speed, force, error)) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    return writeCommand(1, 0, 0, speed, force, error);
  }

  bool moveNormalized(double position, std::string& error) {
    const double clamped = clampDouble(position, 0.0, 1.0);
    const int rPR = clampInt(static_cast<int>(std::lround(clamped * 255.0)), 0, 255);
    const int speed = clampInt(config_.speed, 0, 255);
    const int force = clampInt(config_.force, 0, 255);
    return writeCommand(1, 1, rPR, speed, force, error);
  }

 private:
  struct RobotiqStatus {
    int gACT{0};
    int gSTA{0};
    int gOBJ{0};
    int gFLT{0};
    int gPR{0};
    int gPO{0};
    int gCU{0};
  };

  bool readStatus(RobotiqStatus& status, std::string& error) {
    std::vector<uint8_t> payload;
    if (!readInputRegisters(config_.in_start, 3, payload, error)) {
      return false;
    }
    if (payload.size() < 6) {
      error = "short status payload";
      return false;
    }
    const uint8_t b0 = payload[0];
    const uint8_t b1 = payload[1];
    const uint8_t b2 = payload[2];
    const uint8_t b3 = payload[3];
    const uint8_t b4 = payload[4];
    status.gACT = b0 & 0x01;
    status.gSTA = (b0 >> 4) & 0x03;
    status.gOBJ = (b0 >> 6) & 0x03;
    status.gFLT = b1 & 0x0F;
    status.gPR = b2;
    status.gPO = b3;
    status.gCU = b4;
    return true;
  }

  bool readInputRegisters(int start, int qty, std::vector<uint8_t>& payload, std::string& error) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (!serial_.isOpen()) {
      error = "serial not open";
      return false;
    }
    std::vector<uint8_t> req;
    req.reserve(8);
    req.push_back(static_cast<uint8_t>(config_.slave_id & 0xFF));
    req.push_back(0x04);
    req.push_back(static_cast<uint8_t>((start >> 8) & 0xFF));
    req.push_back(static_cast<uint8_t>(start & 0xFF));
    req.push_back(static_cast<uint8_t>((qty >> 8) & 0xFF));
    req.push_back(static_cast<uint8_t>(qty & 0xFF));
    const uint16_t crc = crc16(req.data(), req.size());
    req.push_back(static_cast<uint8_t>(crc & 0xFF));
    req.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

    if (!serial_.writeAll(req.data(), req.size(), error)) {
      return false;
    }

    std::vector<uint8_t> resp;
    const size_t expected = 5 + static_cast<size_t>(qty) * 2;
    if (!serial_.readExact(expected, std::chrono::milliseconds(config_.timeout_ms), resp, error)) {
      if (error.empty()) {
        error = "short read response";
      }
      return false;
    }
    const uint16_t crc_resp =
        static_cast<uint16_t>(resp[resp.size() - 2] | (resp[resp.size() - 1] << 8));
    const uint16_t crc_calc = crc16(resp.data(), resp.size() - 2);
    if (crc_resp != crc_calc) {
      error = "bad CRC";
      return false;
    }
    if (resp[1] & 0x80) {
      error = "exception response";
      return false;
    }
    if (resp[1] != 0x04) {
      error = "unexpected function";
      return false;
    }
    const uint8_t byte_count = resp[2];
    if (byte_count != static_cast<uint8_t>(qty * 2)) {
      error = "unexpected byte count";
      return false;
    }
    payload.assign(resp.begin() + 3, resp.begin() + 3 + byte_count);
    return true;
  }

  bool writeCommand(int rACT, int rGTO, int rPR, int rSP, int rFR, std::string& error) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (!serial_.isOpen()) {
      error = "serial not open";
      return false;
    }
    const uint8_t b0 = static_cast<uint8_t>((rACT & 1) | ((rGTO & 1) << 3));
    std::array<uint8_t, 6> regs{};
    if (config_.codex_layout) {
      regs = {b0,
              0,
              0,
              static_cast<uint8_t>(rPR & 0xFF),
              static_cast<uint8_t>(rSP & 0xFF),
              static_cast<uint8_t>(rFR & 0xFF)};
    } else {
      regs = {b0,
              static_cast<uint8_t>(rPR & 0xFF),
              static_cast<uint8_t>(rSP & 0xFF),
              static_cast<uint8_t>(rFR & 0xFF),
              0,
              0};
    }

    const uint16_t qty = static_cast<uint16_t>(regs.size() / 2);
    std::vector<uint8_t> req;
    req.reserve(7 + regs.size() + 2);
    req.push_back(static_cast<uint8_t>(config_.slave_id & 0xFF));
    req.push_back(0x10);
    req.push_back(static_cast<uint8_t>((config_.out_start >> 8) & 0xFF));
    req.push_back(static_cast<uint8_t>(config_.out_start & 0xFF));
    req.push_back(static_cast<uint8_t>((qty >> 8) & 0xFF));
    req.push_back(static_cast<uint8_t>(qty & 0xFF));
    req.push_back(static_cast<uint8_t>(regs.size()));
    req.insert(req.end(), regs.begin(), regs.end());
    const uint16_t crc = crc16(req.data(), req.size());
    req.push_back(static_cast<uint8_t>(crc & 0xFF));
    req.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

    if (!serial_.writeAll(req.data(), req.size(), error)) {
      return false;
    }

    std::vector<uint8_t> resp;
    if (!serial_.readExact(8, std::chrono::milliseconds(config_.timeout_ms), resp, error)) {
      if (error.empty()) {
        error = "short write response";
      }
      return false;
    }
    const uint16_t crc_resp =
        static_cast<uint16_t>(resp[resp.size() - 2] | (resp[resp.size() - 1] << 8));
    const uint16_t crc_calc = crc16(resp.data(), resp.size() - 2);
    if (crc_resp != crc_calc) {
      error = "bad CRC";
      return false;
    }
    if (resp[1] & 0x80) {
      error = "exception response";
      return false;
    }
    if (resp[1] != 0x10) {
      error = "unexpected function";
      return false;
    }
    return true;
  }

  RobotiqConfig config_;
  SerialPort serial_;
  std::mutex io_mutex_;
};

struct ThreadGuard {
  explicit ThreadGuard(std::thread* thread) : thread_(thread) {}

  ~ThreadGuard() {
    if (thread_ && thread_->joinable()) {
      thread_->join();
    }
  }

  ThreadGuard(const ThreadGuard&) = delete;
  ThreadGuard& operator=(const ThreadGuard&) = delete;

 private:
  std::thread* thread_{nullptr};
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

double interpolateScalar(const std::vector<double>& time,
                         const std::vector<double>& values,
                         double t) {
  if (time.empty() || values.empty()) {
    return 0.0;
  }
  if (t <= 0.0) {
    return values.front();
  }
  if (t >= time.back()) {
    return values.back();
  }
  auto it = std::lower_bound(time.begin(), time.end(), t);
  if (it == time.begin()) {
    return values.front();
  }
  const size_t idx1 = static_cast<size_t>(std::distance(time.begin(), it));
  const size_t idx0 = idx1 - 1;
  const double t0 = time[idx0];
  const double t1 = time[idx1];
  const double alpha = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;
  return values[idx0] + (values[idx1] - values[idx0]) * alpha;
}

double maxWidthFromTrajectory(const Trajectory& traj) {
  double max_width = 0.0;
  for (double width : traj.gripper_width) {
    max_width = std::max(max_width, width);
  }
  return max_width;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " <robot_ip> <json_path> [time_scale] [start_speed] [gripper_speed]"
                 " [start_delay] [gripper_force]\n";
    return 1;
  }

  const std::string robot_ip = argv[1];
  const std::string json_path = argv[2];
  const double time_scale = (argc >= 4) ? std::stod(argv[3]) : 1.0;
  const double start_speed = (argc >= 5) ? std::stod(argv[4]) : 0.2;
  const double gripper_speed = (argc >= 6) ? std::stod(argv[5]) : 0.1;
  const double start_delay = (argc >= 7) ? std::stod(argv[6]) : 0.5;
  const double gripper_force = (argc >= 8) ? std::stod(argv[7]) : 20.0;

  try {
    Trajectory traj = loadTrajectory(json_path);

    RobotiqConfig robotiq;
    robotiq.port = envString("ROBOTIQ_PORT", robotiq.port);
    robotiq.baud = envInt("ROBOTIQ_BAUD", robotiq.baud);
    robotiq.slave_id = envInt("ROBOTIQ_SLAVE_ID", robotiq.slave_id);
    robotiq.out_start = envInt("ROBOTIQ_OUT_START", robotiq.out_start);
    robotiq.in_start = envInt("ROBOTIQ_IN_START", robotiq.in_start);
    robotiq.timeout_ms = envInt("ROBOTIQ_TIMEOUT_MS", robotiq.timeout_ms);
    robotiq.min_command_interval_ms =
        envInt("ROBOTIQ_MIN_CMD_MS", robotiq.min_command_interval_ms);
    robotiq.source_max_width = envDouble("ROBOTIQ_SOURCE_MAX_WIDTH", robotiq.source_max_width);
    robotiq.position_eps = envDouble("ROBOTIQ_POSITION_EPS", robotiq.position_eps);
    robotiq.speed = envRobotiqByte("ROBOTIQ_SPEED", toRobotiqByte(gripper_speed, 128));
    robotiq.force = envRobotiqByte("ROBOTIQ_FORCE", toRobotiqByte(gripper_force, 128));
    const std::string layout = toLower(envString("ROBOTIQ_LAYOUT", "codex"));
    robotiq.codex_layout =
        !(layout == "gello" || layout == "legacy" || layout == "original");
    const std::string start_mode = toLower(envString("ROBOTIQ_START_MODE", "follow"));
    robotiq.hold_initial = (start_mode == "hold");
    robotiq.activation_mode = parseActivationMode(envString("ROBOTIQ_ACTIVATE", "auto"));

    if (robotiq.port.empty()) {
      throw std::runtime_error("ROBOTIQ_PORT is empty");
    }
    const double traj_max_width = maxWidthFromTrajectory(traj);
    double source_max_width = robotiq.source_max_width;
    if (source_max_width <= 0.0) {
      source_max_width = traj_max_width;
    }
    if (source_max_width <= 0.0) {
      throw std::runtime_error("ROBOTIQ_SOURCE_MAX_WIDTH must be positive");
    }

    RobotiqModbus gripper(robotiq);

    std::cout << "Waiting " << start_delay << " s for arm alignment..." << std::endl;
    if (start_delay > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(start_delay));
    }

    std::cout << "Connecting Robotiq gripper..." << std::endl;
    gripper.connect();
    std::string cmd_error;
    if (!gripper.activate(cmd_error)) {
      throw std::runtime_error("Robotiq activate failed: " + cmd_error);
    }

    const double duration = traj.time.back();
    const auto min_command_interval =
        std::chrono::milliseconds(robotiq.min_command_interval_ms);
    const auto start = std::chrono::steady_clock::now();
    auto last_cmd_time = start - min_command_interval;
    double last_position = -1.0;
    bool have_state = false;
    std::atomic<bool> gripper_busy{false};
    std::exception_ptr gripper_error = nullptr;
    std::mutex gripper_error_mutex;
    std::thread cmd_thread;
    ThreadGuard cmd_guard(&cmd_thread);
    bool initial_skipped = false;

    auto start_command = [&](double position) {
      if (cmd_thread.joinable()) {
        cmd_thread.join();
      }
      gripper_busy.store(true);
      cmd_thread = std::thread([&, position]() {
        try {
          std::string error;
          if (!gripper.moveNormalized(position, error)) {
            throw std::runtime_error("Robotiq command failed: " + error);
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
      const double width = interpolateScalar(traj.time, traj.gripper_width, t_clamped);
      const double width_ratio =
          clampDouble(width / std::max(1e-6, source_max_width), 0.0, 1.0);
      const double position = 1.0 - width_ratio;

      if (!have_state) {
        if (robotiq.hold_initial && !initial_skipped) {
          last_position = position;
          have_state = true;
          initial_skipped = true;
        }
      }

      if (!have_state || std::abs(position - last_position) >= robotiq.position_eps) {
        if (!gripper_busy.load() &&
            now - last_cmd_time >= min_command_interval) {
          start_command(position);
          last_cmd_time = now;
          last_position = position;
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
    gripper.close();
    (void)robot_ip;
    (void)start_speed;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 3;
  }

  return 0;
}
