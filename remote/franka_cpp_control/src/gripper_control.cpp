#include <chrono>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>

namespace {

void printUsage(const char* argv0) {
  std::cerr << "Usage:\n"
            << "  " << argv0 << " <robot_ip> home\n"
            << "  " << argv0 << " <robot_ip> open [width] [speed]\n"
            << "  " << argv0 << " <robot_ip> close [width] [speed] [force]\n"
            << "  " << argv0
            << " <robot_ip> grasp [width] [speed] [force] [eps_in] [eps_out]\n"
            << "  " << argv0 << " <robot_ip> state\n"
            << "  " << argv0 << " <robot_ip> pulse [open_width] [speed] [force]\n";
}

double parseOrDefault(int argc, char** argv, int index, double fallback) {
  if (index < argc) {
    return std::stod(argv[index]);
  }
  return fallback;
}

void printState(const franka::GripperState& state) {
  std::cout << "Width: " << state.width << " m\n"
            << "Max width: " << state.max_width << " m\n"
            << "Is grasped: " << (state.is_grasped ? "true" : "false") << "\n"
            << "Temperature: " << state.temperature << " C\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    printUsage(argv[0]);
    return 1;
  }

  const std::string robot_ip = argv[1];
  const std::string command = argv[2];

  try {
    franka::Gripper gripper(robot_ip);

    if (command == "home") {
      bool ok = gripper.homing();
      std::cout << "Homing: " << (ok ? "success" : "failed") << std::endl;
      return ok ? 0 : 2;
    }

    if (command == "open") {
      const double width = parseOrDefault(argc, argv, 3, 0.08);
      const double speed = parseOrDefault(argc, argv, 4, 0.1);
      bool ok = gripper.move(width, speed);
      std::cout << "Open to " << width << " m: " << (ok ? "success" : "failed")
                << std::endl;
      return ok ? 0 : 2;
    }

    if (command == "close") {
      const double width = parseOrDefault(argc, argv, 3, 0.0);
      const double speed = parseOrDefault(argc, argv, 4, 0.1);
      const double force = parseOrDefault(argc, argv, 5, 20.0);
      bool ok = gripper.grasp(width, speed, force, 0.005, 0.005);
      std::cout << "Close to " << width << " m: " << (ok ? "success" : "failed")
                << std::endl;
      return ok ? 0 : 2;
    }

    if (command == "grasp") {
      const double width = parseOrDefault(argc, argv, 3, 0.02);
      const double speed = parseOrDefault(argc, argv, 4, 0.1);
      const double force = parseOrDefault(argc, argv, 5, 20.0);
      const double eps_in = parseOrDefault(argc, argv, 6, 0.005);
      const double eps_out = parseOrDefault(argc, argv, 7, 0.005);
      bool ok = gripper.grasp(width, speed, force, eps_in, eps_out);
      std::cout << "Grasp at " << width << " m: " << (ok ? "success" : "failed")
                << std::endl;
      return ok ? 0 : 2;
    }

    if (command == "state") {
      auto state = gripper.readOnce();
      printState(state);
      return 0;
    }

    if (command == "pulse") {
      const double open_width = parseOrDefault(argc, argv, 3, gripper.readOnce().max_width);
      const double speed = parseOrDefault(argc, argv, 4, 0.1);
      const double force = parseOrDefault(argc, argv, 5, 20.0);
      bool ok_open = gripper.move(open_width, speed);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      bool ok_close = gripper.grasp(0.0, speed, force, 0.005, 0.005);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      std::cout << "Pulse open/close: "
                << ((ok_open && ok_close) ? "success" : "failed") << std::endl;
      return (ok_open && ok_close) ? 0 : 2;
    }

    std::cerr << "Unknown command: " << command << std::endl;
    printUsage(argv[0]);
    return 1;
  } catch (const franka::Exception& e) {
    std::cerr << "libfranka exception: " << e.what() << std::endl;
    return 2;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 3;
  }
}
