#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <cstdint>
#include <csignal>
#include <cstring>
#include <ctime>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

namespace {

std::atomic<bool> g_running{true};

void signalHandler(int) {
  g_running.store(false);
}

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

bool ensureDir(const std::string& dir) {
  if (dir.empty()) {
    return false;
  }
  struct stat st {};
  if (::stat(dir.c_str(), &st) == 0) {
    return S_ISDIR(st.st_mode);
  }
  if (::mkdir(dir.c_str(), 0755) == 0) {
    return true;
  }
  if (errno == EEXIST) {
    return true;
  }
  return false;
}

std::string makeTimestamp() {
  std::time_t now = std::time(nullptr);
  std::tm local_tm{};
  localtime_r(&now, &local_tm);
  char buffer[32];
  if (std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &local_tm) == 0) {
    return "unknown_time";
  }
  return std::string(buffer);
}

double nowSeconds() {
  const auto now = std::chrono::system_clock::now();
  return std::chrono::duration<double>(now.time_since_epoch()).count();
}

class ActionLogger {
 public:
  ActionLogger() = default;

  void init(const std::string& dir) {
    if (!ensureDir(dir)) {
      std::cerr << "Action log dir unavailable: " << dir << std::endl;
      return;
    }
    const std::string path = dir + "/" + makeTimestamp() + ".log";
    file_.open(path, std::ios::out | std::ios::app);
    if (!file_.is_open()) {
      std::cerr << "Failed to open action log: " << path << std::endl;
      return;
    }
    file_.setf(std::ios::fixed);
    file_ << std::setprecision(6);
    enabled_ = true;
    std::cerr << "Action log: " << path << std::endl;
    file_ << "{\"type\":\"start\",\"t\":" << nowSeconds() << "}\n";
    file_.flush();
  }

  void logAction(const char* type, const std::array<double, 8>& action) {
    if (!enabled_) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    file_ << "{\"type\":\"" << type << "\",\"t\":" << nowSeconds() << ",\"action\":[";
    for (size_t i = 0; i < action.size(); ++i) {
      if (i > 0) {
        file_ << ",";
      }
      file_ << action[i];
    }
    file_ << "]}\n";
    file_.flush();
  }

 private:
  std::mutex mutex_;
  std::ofstream file_;
  bool enabled_{false};
};

struct ActionSample {
  double t{0.0};
  std::array<double, 8> action{};
};

struct ActionBuffer {
  std::mutex mutex;
  std::deque<ActionSample> samples;
  double next_time{0.0};
  double sample_dt{1.0};
  std::chrono::steady_clock::time_point last_update{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point start_time{};
  bool has_start_time{false};
};

struct StateCache {
  std::mutex mutex;
  std::array<double, 7> joint{};
  double gripper{0.0};
  double timestamp{0.0};
  bool valid{false};
};

bool parseActionsJson(const std::string& line, std::vector<std::array<double, 8>>& actions) {
  std::stringstream ss(line);
  boost::property_tree::ptree tree;
  try {
    boost::property_tree::read_json(ss, tree);
  } catch (const std::exception& e) {
    std::cerr << "Failed to parse action json: " << e.what() << std::endl;
    return false;
  }

  actions.clear();
  auto actions_node = tree.get_child_optional("actions");
  if (!actions_node) {
    return false;
  }

  for (const auto& item : actions_node.value()) {
    std::array<double, 8> action{};
    size_t idx = 0;
    for (const auto& v : item.second) {
      if (idx >= action.size()) {
        break;
      }
      action[idx] = v.second.get_value<double>();
      ++idx;
    }
    actions.push_back(action);
  }
  return !actions.empty();
}

class ActionReceiver {
 public:
  ActionReceiver(int port, ActionBuffer& buffer, ActionLogger* logger)
      : port_(port), buffer_(buffer), logger_(logger) {}

  void start() { thread_ = std::thread(&ActionReceiver::run, this); }

  void stop() {
    if (server_fd_ >= 0) {
      ::shutdown(server_fd_, SHUT_RDWR);
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }

 private:
  void run() {
    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
      std::cerr << "Failed to create action socket: " << std::strerror(errno) << std::endl;
      return;
    }

    int opt = 1;
    ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(port_));

    if (::bind(server_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      std::cerr << "Failed to bind action socket: " << std::strerror(errno) << std::endl;
      return;
    }

    if (::listen(server_fd_, 1) < 0) {
      std::cerr << "Failed to listen action socket: " << std::strerror(errno) << std::endl;
      return;
    }

    while (g_running.load()) {
      sockaddr_in client{};
      socklen_t len = sizeof(client);
      const int client_fd = ::accept(server_fd_, reinterpret_cast<sockaddr*>(&client), &len);
      if (client_fd < 0) {
        if (!g_running.load()) {
          break;
        }
        continue;
      }

      std::string buffer;
      char temp[4096];
      while (g_running.load()) {
        const ssize_t got = ::recv(client_fd, temp, sizeof(temp), 0);
        if (got <= 0) {
          break;
        }
        buffer.append(temp, temp + got);
        size_t pos = 0;
        while ((pos = buffer.find('\n')) != std::string::npos) {
          std::string line = buffer.substr(0, pos);
          buffer.erase(0, pos + 1);
          if (line.empty()) {
            continue;
          }
          std::vector<std::array<double, 8>> actions;
          if (!parseActionsJson(line, actions)) {
            continue;
          }
          double max_abs = 0.0;
          for (const auto& action : actions) {
            for (double v : action) {
              max_abs = std::max(max_abs, std::abs(v));
            }
          }
          std::cerr << "Received action chunk size=" << actions.size()
                    << ", max_abs=" << max_abs << std::endl;
          if (logger_) {
            for (const auto& action : actions) {
              logger_->logAction("recv", action);
            }
          }
          std::lock_guard<std::mutex> lock(buffer_.mutex);
          const auto now_tp = std::chrono::steady_clock::now();
          if (!buffer_.has_start_time) {
            buffer_.start_time = now_tp;
            buffer_.has_start_time = true;
          }
          for (const auto& action : actions) {
            ActionSample sample;
            sample.t = buffer_.next_time;
            sample.action = action;
            buffer_.samples.push_back(sample);
            buffer_.next_time += buffer_.sample_dt;
          }
          buffer_.last_update = now_tp;
        }
      }
      ::close(client_fd);
    }

    if (server_fd_ >= 0) {
      ::close(server_fd_);
      server_fd_ = -1;
    }
  }

  int port_{15123};
  ActionBuffer& buffer_;
  ActionLogger* logger_{nullptr};
  std::thread thread_;
  int server_fd_{-1};
};

class StateServer {
 public:
  StateServer(int port, StateCache& cache, double rate_hz)
      : port_(port), cache_(cache), rate_hz_(rate_hz) {}

  void start() { thread_ = std::thread(&StateServer::run, this); }

  void stop() {
    if (server_fd_ >= 0) {
      ::shutdown(server_fd_, SHUT_RDWR);
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }

 private:
  void run() {
    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
      std::cerr << "Failed to create state socket: " << std::strerror(errno) << "\n";
      return;
    }

    int opt = 1;
    ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(port_));

    if (::bind(server_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      std::cerr << "Failed to bind state socket: " << std::strerror(errno) << "\n";
      return;
    }

    if (::listen(server_fd_, 1) < 0) {
      std::cerr << "Failed to listen state socket: " << std::strerror(errno) << "\n";
      return;
    }

    const double interval_s = 1.0 / std::max(1e-6, rate_hz_);

    while (g_running.load()) {
      sockaddr_in client{};
      socklen_t len = sizeof(client);
      const int client_fd = ::accept(server_fd_, reinterpret_cast<sockaddr*>(&client), &len);
      if (client_fd < 0) {
        if (!g_running.load()) {
          break;
        }
        continue;
      }

      while (g_running.load()) {
        std::array<double, 7> joint{};
        double gripper = 0.0;
        double timestamp = 0.0;
        {
          std::lock_guard<std::mutex> lock(cache_.mutex);
          if (!cache_.valid) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
          }
          joint = cache_.joint;
          gripper = cache_.gripper;
          timestamp = cache_.timestamp;
        }
        std::ostringstream ss;
        ss << "{\"timestamp\":" << timestamp << ",\"joint_position\":[";
        for (size_t i = 0; i < joint.size(); ++i) {
          if (i > 0) {
            ss << ",";
          }
          ss << joint[i];
        }
        ss << "],\"gripper_position\":" << gripper << "}\n";
        const std::string payload = ss.str();
        const ssize_t sent = ::send(client_fd, payload.data(), payload.size(), 0);
        if (sent <= 0) {
          break;
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(interval_s));
      }
      ::close(client_fd);
    }

    if (server_fd_ >= 0) {
      ::close(server_fd_);
      server_fd_ = -1;
    }
  }

  int port_{15124};
  StateCache& cache_;
  double rate_hz_{30.0};
  std::thread thread_;
  int server_fd_{-1};
};

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
  int activation_mode{0};
};

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

int clampInt(int value, int low, int high) {
  return std::max(low, std::min(high, value));
}

double clampDouble(double value, double low, double high) {
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
    }
    fd_ = -1;
  }

  bool writeAll(const uint8_t* data, size_t length, std::string& error) {
    size_t sent = 0;
    while (sent < length) {
      const ssize_t rc = ::write(fd_, data + sent, length - sent);
      if (rc < 0) {
        error = std::strerror(errno);
        return false;
      }
      sent += static_cast<size_t>(rc);
    }
    return true;
  }

  bool readExact(size_t length,
                 std::chrono::milliseconds timeout,
                 std::vector<uint8_t>& out,
                 std::string& error) {
    out.clear();
    out.reserve(length);
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (out.size() < length) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        break;
      }
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
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

RobotiqConfig loadRobotiqConfig() {
  RobotiqConfig robotiq;
  robotiq.port = envString("ROBOTIQ_PORT", robotiq.port);
  robotiq.baud = envInt("ROBOTIQ_BAUD", robotiq.baud);
  robotiq.slave_id = envInt("ROBOTIQ_SLAVE_ID", robotiq.slave_id);
  robotiq.out_start = envInt("ROBOTIQ_OUT_START", robotiq.out_start);
  robotiq.in_start = envInt("ROBOTIQ_IN_START", robotiq.in_start);
  robotiq.timeout_ms = envInt("ROBOTIQ_TIMEOUT_MS", robotiq.timeout_ms);
  robotiq.min_command_interval_ms = envInt("ROBOTIQ_MIN_CMD_MS", robotiq.min_command_interval_ms);
  robotiq.source_max_width = envDouble("ROBOTIQ_SOURCE_MAX_WIDTH", robotiq.source_max_width);
  robotiq.position_eps = envDouble("ROBOTIQ_POSITION_EPS", robotiq.position_eps);
  robotiq.speed = envRobotiqByte("ROBOTIQ_SPEED", robotiq.speed);
  robotiq.force = envRobotiqByte("ROBOTIQ_FORCE", robotiq.force);
  const std::string layout = toLower(envString("ROBOTIQ_LAYOUT", "codex"));
  robotiq.codex_layout = !(layout == "gello" || layout == "legacy" || layout == "original");
  robotiq.activation_mode = parseActivationMode(envString("ROBOTIQ_ACTIVATE", "auto"));
  return robotiq;
}

std::array<int, 7> buildActionMap(const std::string& order_csv, bool& valid) {
  const std::array<std::string, 7> canonical = {
      "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
      "fr3_joint5", "fr3_joint6", "fr3_joint7"};
  std::array<int, 7> mapping{};
  valid = false;

  if (order_csv.empty()) {
    for (size_t i = 0; i < canonical.size(); ++i) {
      mapping[i] = static_cast<int>(i);
    }
    return mapping;
  }

  std::vector<std::string> order;
  std::string token;
  std::stringstream ss(order_csv);
  while (std::getline(ss, token, ',')) {
    if (!token.empty()) {
      order.push_back(token);
    }
  }
  if (order.size() != canonical.size()) {
    std::cerr << "ACTION_ORDER size mismatch; expected 7, got " << order.size() << std::endl;
    for (size_t i = 0; i < canonical.size(); ++i) {
      mapping[i] = static_cast<int>(i);
    }
    return mapping;
  }

  for (size_t i = 0; i < canonical.size(); ++i) {
    auto it = std::find(order.begin(), order.end(), canonical[i]);
    if (it == order.end()) {
      std::cerr << "ACTION_ORDER missing " << canonical[i] << std::endl;
      for (size_t j = 0; j < canonical.size(); ++j) {
        mapping[j] = static_cast<int>(j);
      }
      return mapping;
    }
    mapping[i] = static_cast<int>(std::distance(order.begin(), it));
  }

  valid = true;
  return mapping;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <robot_ip> [action_port] [state_port] [rate_hz] [max_acc] [vel_alpha]"
                 " [timeout_s] [invert_gripper] [kp] [time_scale]\n";
    return 1;
  }

  const std::string robot_ip = argv[1];
  const int action_port = (argc >= 3) ? std::stoi(argv[2]) : 15123;
  const int state_port = (argc >= 4) ? std::stoi(argv[3]) : action_port + 1;
  const double rate_hz = (argc >= 5) ? std::stod(argv[4]) : 15.0;
  const double max_acc = (argc >= 6) ? std::stod(argv[5]) : 5.0;
  const double vel_alpha = (argc >= 7) ? std::stod(argv[6]) : 0.9;
  const double timeout_s = (argc >= 8) ? std::stod(argv[7]) : 2.0;
  const bool invert_gripper = (argc >= 9) ? (std::stoi(argv[8]) != 0) : false;
  const double kp = (argc >= 10) ? std::stod(argv[9]) : 2.0;
  const double time_scale = (argc >= 11) ? std::stod(argv[10]) : 1.0;
  (void)kp;
  (void)rate_hz;

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  ActionBuffer action_buffer;
  StateCache state_cache;

  ActionLogger action_logger;
  action_logger.init(envString("ACTION_LOG_DIR", "./action_buffer"));

  bool action_map_valid = false;
  const std::string action_order = envString("ACTION_ORDER", "");
  const std::array<int, 7> action_map = buildActionMap(action_order, action_map_valid);
  if (!action_order.empty() && action_map_valid) {
    std::cerr << "Using ACTION_ORDER mapping: " << action_order << std::endl;
  }

  ActionReceiver receiver(action_port, action_buffer, &action_logger);
  receiver.start();

  StateServer state_server(state_port, state_cache, 30.0);
  state_server.start();

  std::atomic<double> gripper_target{-1.0};

  std::thread gripper_thread([&]() {
    RobotiqConfig robotiq = loadRobotiqConfig();
    if (robotiq.port.empty()) {
      std::cerr << "ROBOTIQ_PORT is empty; gripper disabled." << std::endl;
      return;
    }
    try {
      RobotiqModbus gripper(robotiq);
      gripper.connect();
      std::string error;
      if (!gripper.activate(error)) {
        std::cerr << "Robotiq activate failed: " << error << std::endl;
        return;
      }
      auto last_cmd_time = std::chrono::steady_clock::now() -
                           std::chrono::milliseconds(robotiq.min_command_interval_ms);
      double last_position = -1.0;
      while (g_running.load()) {
        const double target = gripper_target.load();
        if (target < 0.0) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          continue;
        }
        double position = clampDouble(target, 0.0, 1.0);
        if (invert_gripper) {
          position = 1.0 - position;
        }
        const auto now = std::chrono::steady_clock::now();
        if (std::abs(position - last_position) >= robotiq.position_eps &&
            now - last_cmd_time >= std::chrono::milliseconds(robotiq.min_command_interval_ms)) {
          std::string err;
          if (!gripper.moveNormalized(position, err)) {
            std::cerr << "Robotiq move failed: " << err << std::endl;
          } else {
            last_position = position;
            last_cmd_time = now;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      gripper.close();
    } catch (const std::exception& e) {
      std::cerr << "Gripper thread error: " << e.what() << std::endl;
    }
  });

  try {
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);
    robot.automaticErrorRecovery();

    const double action_dt = 1.0 / 30.0;
    const double scale = 1.0 / std::max(1e-6, time_scale);
    action_buffer.sample_dt = action_dt;
    std::array<double, 8> current_action{};
    std::array<double, 7> dq_filtered{};
    std::array<double, 7> last_dq{};
    std::array<double, 7> last_ddq{};
    double stream_time = 0.0;

    auto control_callback = [&](const franka::RobotState& state,
                                franka::Duration period) -> franka::JointVelocities {
      const double dt = std::max(1e-6, period.toSec());
      const auto now = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> lock(action_buffer.mutex);
        if (action_buffer.has_start_time) {
          stream_time = std::chrono::duration<double>(now - action_buffer.start_time).count();
        } else {
          stream_time += dt;
        }
      }
      {
        std::lock_guard<std::mutex> lock(action_buffer.mutex);
        const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
            now - action_buffer.last_update);
        if (action_buffer.samples.empty() || elapsed.count() > timeout_s) {
          current_action.fill(0.0);
        } else {
          while (action_buffer.samples.size() >= 2 &&
                 action_buffer.samples[1].t <= stream_time) {
            action_buffer.samples.pop_front();
          }
          const auto& a0 = action_buffer.samples.front();
          const auto& a1 = (action_buffer.samples.size() >= 2)
                               ? action_buffer.samples[1]
                               : action_buffer.samples.front();
          const double denom = std::max(1e-6, a1.t - a0.t);
          const double alpha =
              (stream_time <= a0.t) ? 0.0 : std::min(1.0, (stream_time - a0.t) / denom);
          for (size_t i = 0; i < current_action.size(); ++i) {
            current_action[i] =
                a0.action[i] + (a1.action[i] - a0.action[i]) * alpha;
          }
        }
      }
      std::array<double, 8> mapped_action = current_action;
      for (size_t i = 0; i < 7; ++i) {
        const int src = action_map[i];
        if (src >= 0 && src < 7) {
          mapped_action[i] = current_action[static_cast<size_t>(src)];
        }
      }
      action_logger.logAction("exec", mapped_action);
      current_action = mapped_action;
      gripper_target.store(current_action[7]);

      {
        std::lock_guard<std::mutex> lock(state_cache.mutex);
        state_cache.joint = state.q;
        state_cache.gripper = clampDouble(gripper_target.load(), 0.0, 1.0);
        state_cache.timestamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        state_cache.valid = true;
      }

      std::array<double, 7> dq_cmd{};
      for (size_t i = 0; i < 7; ++i) {
        const double dq_raw = current_action[i] * scale;
        dq_filtered[i] = vel_alpha * dq_filtered[i] + (1.0 - vel_alpha) * dq_raw;
        dq_cmd[i] = dq_filtered[i];
      }

      if (stream_time < 1.0) {
        const double w = smoothStep(stream_time / 1.0);
        for (size_t i = 0; i < 7; ++i) {
          dq_cmd[i] *= w;
        }
      }

      const auto upper = franka::computeUpperLimitsJointVelocity(state.q_d);
      const auto lower = franka::computeLowerLimitsJointVelocity(state.q_d);
      const double max_jerk = std::max(1.0, max_acc * 10.0);
      std::array<double, 7> dq_limited{};
      for (size_t i = 0; i < 7; ++i) {
        const double dq_smoothed = vel_alpha * last_dq[i] + (1.0 - vel_alpha) * dq_cmd[i];
        dq_limited[i] = franka::limitRate(upper[i],
                                          lower[i],
                                          max_acc,
                                          max_jerk,
                                          dq_smoothed,
                                          last_dq[i],
                                          last_ddq[i]);
      }
      for (size_t i = 0; i < 7; ++i) {
        last_ddq[i] = (dq_limited[i] - last_dq[i]) / dt;
        last_dq[i] = dq_limited[i];
      }

      if (!g_running.load()) {
        return franka::MotionFinished(franka::JointVelocities(std::array<double, 7>{}));
      }
      return franka::JointVelocities(dq_limited);
    };

    std::cout << "Waiting for action stream on port " << action_port
              << ", state on port " << state_port << "..." << std::endl;
    robot.control(control_callback);
  } catch (const franka::Exception& e) {
    std::cerr << "libfranka exception: " << e.what() << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
  }

  g_running.store(false);
  receiver.stop();
  state_server.stop();
  if (gripper_thread.joinable()) {
    gripper_thread.join();
  }

  return 0;
}
