cmake_minimum_required(VERSION 3.10)
project(franka_cpp_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(LOCAL_LIBFRANKA_PREFIX "${CMAKE_CURRENT_LIST_DIR}/third_party/libfranka-0.18.0")
if(EXISTS "${LOCAL_LIBFRANKA_PREFIX}/lib/cmake/Franka/FrankaConfig.cmake")
  list(PREPEND CMAKE_PREFIX_PATH "${LOCAL_LIBFRANKA_PREFIX}")
endif()

find_package(Franka 0.18.0 REQUIRED)

add_executable(franka_ready_pose src/main.cpp)
target_link_libraries(franka_ready_pose Franka::Franka)
set_target_properties(franka_ready_pose PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)

add_executable(franka_circle_xy src/circle_xy.cpp)
target_link_libraries(franka_circle_xy Franka::Franka)
set_target_properties(franka_circle_xy PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)

add_executable(franka_gripper_control src/gripper_control.cpp)
target_link_libraries(franka_gripper_control Franka::Franka)
set_target_properties(franka_gripper_control PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)

set(BOOST_INCLUDE_HINTS
  /home/rsj/anaconda3/pkgs/libboost-headers-1.88.0-ha770c72_7/include
  /home/rsj/anaconda3/pkgs/libboost-headers-1.88.0-h06a4308_0/include
  /home/rsj/anaconda3/pkgs/libboost-1.82.0-h109eef0_2/include
  /home/rsj/anaconda3/include
)
set(BOOST_INCLUDE_DIR "")
foreach(path IN LISTS BOOST_INCLUDE_HINTS)
  if(EXISTS "${path}/boost/property_tree/json_parser.hpp")
    set(BOOST_INCLUDE_DIR "${path}")
    break()
  endif()
endforeach()

add_executable(franka_track_json src/track_json.cpp)
target_link_libraries(franka_track_json Franka::Franka)
set_target_properties(franka_track_json PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)
if(BOOST_INCLUDE_DIR)
  target_include_directories(franka_track_json PRIVATE "${BOOST_INCLUDE_DIR}")
endif()

add_executable(franka_track_json_raw src/track_json_raw.cpp)
target_link_libraries(franka_track_json_raw Franka::Franka)
set_target_properties(franka_track_json_raw PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)
if(BOOST_INCLUDE_DIR)
  target_include_directories(franka_track_json_raw PRIVATE "${BOOST_INCLUDE_DIR}")
endif()

add_executable(franka_track_json_vel src/track_json_vel.cpp)
target_link_libraries(franka_track_json_vel Franka::Franka)
set_target_properties(franka_track_json_vel PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)
if(BOOST_INCLUDE_DIR)
  target_include_directories(franka_track_json_vel PRIVATE "${BOOST_INCLUDE_DIR}")
endif()

add_executable(franka_track_json_vel_smooth src/track_json_vel_smooth.cpp)
target_link_libraries(franka_track_json_vel_smooth Franka::Franka)
set_target_properties(franka_track_json_vel_smooth PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)
if(BOOST_INCLUDE_DIR)
  target_include_directories(franka_track_json_vel_smooth PRIVATE "${BOOST_INCLUDE_DIR}")
endif()

add_executable(franka_track_json_vel_from_json src/track_json_vel_from_json.cpp)
target_link_libraries(franka_track_json_vel_from_json Franka::Franka)
set_target_properties(franka_track_json_vel_from_json PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)
if(BOOST_INCLUDE_DIR)
  target_include_directories(franka_track_json_vel_from_json PRIVATE "${BOOST_INCLUDE_DIR}")
endif()

add_executable(franka_track_json_vel_only_from_json src/track_json_vel_only_from_json.cpp)
target_link_libraries(franka_track_json_vel_only_from_json Franka::Franka)
set_target_properties(franka_track_json_vel_only_from_json PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)
if(BOOST_INCLUDE_DIR)
  target_include_directories(franka_track_json_vel_only_from_json PRIVATE "${BOOST_INCLUDE_DIR}")
endif()

add_executable(franka_track_json_impedance src/track_json_impedance.cpp)
target_link_libraries(franka_track_json_impedance Franka::Franka)
set_target_properties(franka_track_json_impedance PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)
if(BOOST_INCLUDE_DIR)
  target_include_directories(franka_track_json_impedance PRIVATE "${BOOST_INCLUDE_DIR}")
endif()

add_executable(franka_track_json_impedance_smooth src/track_json_impedance_smooth.cpp)
target_link_libraries(franka_track_json_impedance_smooth Franka::Franka)
set_target_properties(franka_track_json_impedance_smooth PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)
if(BOOST_INCLUDE_DIR)
  target_include_directories(franka_track_json_impedance_smooth PRIVATE "${BOOST_INCLUDE_DIR}")
endif()

add_executable(franka_track_json_gripper_only
  src/track_json_gripper_only.cpp
)
target_link_libraries(franka_track_json_gripper_only Franka::Franka)
set_target_properties(franka_track_json_gripper_only PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)
if(BOOST_INCLUDE_DIR)
  target_include_directories(franka_track_json_gripper_only PRIVATE "${BOOST_INCLUDE_DIR}")
endif()

add_executable(franka_track_json_gripper_only_robotiq
  src/track_json_gripper_only_robotiq.cpp
)
set_target_properties(franka_track_json_gripper_only_robotiq PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)
if(BOOST_INCLUDE_DIR)
  target_include_directories(franka_track_json_gripper_only_robotiq PRIVATE "${BOOST_INCLUDE_DIR}")
endif()

add_executable(franka_track_json_impedance_gripper
  src/track_json_impedance_gripper.cpp
)
target_link_libraries(franka_track_json_impedance_gripper Franka::Franka)
set_target_properties(franka_track_json_impedance_gripper PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)
if(BOOST_INCLUDE_DIR)
  target_include_directories(franka_track_json_impedance_gripper PRIVATE "${BOOST_INCLUDE_DIR}")
endif()

add_executable(franka_track_json_impedance_gripper_record
  src/track_json_impedance_gripper_record.cpp
)
target_link_libraries(franka_track_json_impedance_gripper_record Franka::Franka)
set_target_properties(franka_track_json_impedance_gripper_record PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)
if(BOOST_INCLUDE_DIR)
  target_include_directories(franka_track_json_impedance_gripper_record PRIVATE "${BOOST_INCLUDE_DIR}")
endif()

add_executable(franka_track_json_impedance_gripper_raw
  src/track_json_impedance_gripper_raw.cpp
)
target_link_libraries(franka_track_json_impedance_gripper_raw Franka::Franka)
set_target_properties(franka_track_json_impedance_gripper_raw PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)
if(BOOST_INCLUDE_DIR)
  target_include_directories(franka_track_json_impedance_gripper_raw PRIVATE "${BOOST_INCLUDE_DIR}")
endif()

add_executable(franka_openpi_stream_vel_robotiq
  src/openpi_stream_vel_robotiq.cpp
)
target_link_libraries(franka_openpi_stream_vel_robotiq Franka::Franka)
set_target_properties(franka_openpi_stream_vel_robotiq PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)
if(BOOST_INCLUDE_DIR)
  target_include_directories(franka_openpi_stream_vel_robotiq PRIVATE "${BOOST_INCLUDE_DIR}")
endif()

add_executable(franka_openpi_stream_vel_robotiq_trackjson_30hz
  src/openpi_stream_vel_robotiq_trackjson_30hz.cpp
)
target_link_libraries(franka_openpi_stream_vel_robotiq_trackjson_30hz Franka::Franka)
set_target_properties(franka_openpi_stream_vel_robotiq_trackjson_30hz PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)
if(BOOST_INCLUDE_DIR)
  target_include_directories(franka_openpi_stream_vel_robotiq_trackjson_30hz PRIVATE "${BOOST_INCLUDE_DIR}")
endif()

add_executable(franka_openpi_stream_impedance_pos_robotiq
  src/openpi_stream_impedance_pos_robotiq.cpp
)
target_link_libraries(franka_openpi_stream_impedance_pos_robotiq Franka::Franka)
set_target_properties(franka_openpi_stream_impedance_pos_robotiq PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)
if(BOOST_INCLUDE_DIR)
  target_include_directories(franka_openpi_stream_impedance_pos_robotiq PRIVATE "${BOOST_INCLUDE_DIR}")
endif()

add_executable(franka_openpi_stream_vel_robotiq_direct
  src/openpi_stream_vel_robotiq_direct.cpp
)
target_link_libraries(franka_openpi_stream_vel_robotiq_direct Franka::Franka)
set_target_properties(franka_openpi_stream_vel_robotiq_direct PROPERTIES
  BUILD_RPATH "${LOCAL_LIBFRANKA_PREFIX}/lib"
)
if(BOOST_INCLUDE_DIR)
  target_include_directories(franka_openpi_stream_vel_robotiq_direct PRIVATE "${BOOST_INCLUDE_DIR}")
endif()
