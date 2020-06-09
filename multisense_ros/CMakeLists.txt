cmake_minimum_required(VERSION 3.5)
project(multisense_ros)

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
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(multisense_lib REQUIRED)
find_package(multisense_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} src/camera.cpp
                            src/camera_utilities.cpp
                            src/config.cpp
                            src/imu.cpp
                            src/laser.cpp
                            src/pps.cpp
                            src/status.cpp)

ament_target_dependencies(${PROJECT_NAME}
                         geometry_msgs
                         image_transport
                         multisense_lib
                         multisense_msgs
                         OpenCV
                         rclcpp
                         sensor_msgs
                         stereo_msgs
                         tf2
                         tf2_ros)


add_executable(ros_driver src/ros_driver.cpp)
target_link_libraries(ros_driver ${PROJECT_NAME})

ament_target_dependencies(ros_driver
                         rclcpp)

install(TARGETS ros_driver
        DESTINATION lib/${PROJECT_NAME})

ament_package()
