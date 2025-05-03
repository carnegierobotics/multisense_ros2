cmake_minimum_required(VERSION 3.5)

project(multisense_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Selecting default build type: Release")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default: Release)" FORCE)
endif ()

find_package(ament_cmake REQUIRED)

# Message generation
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
    "msg/Histogram.msg"
    "msg/Info.msg"
    "msg/Status.msg")

rosidl_generate_interfaces(${PROJECT_NAME}
                           ${msg_files}
                           DEPENDENCIES builtin_interfaces)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

