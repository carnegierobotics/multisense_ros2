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
find_package(angles REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(multisense_lib REQUIRED)
find_package(multisense_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_environment REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} src/camera.cpp
                            src/camera_utilities.cpp
                            src/config.cpp
                            src/imu.cpp
                            src/laser.cpp
                            src/parameter_utilities.cpp
                            src/point_cloud_utilities.cpp
                            src/pps.cpp
                            src/status.cpp)

string(TOUPPER "ROS_$ENV{ROS_DISTRO}" ROS_DISTRO)
target_compile_definitions(${PROJECT_NAME} PRIVATE ${ROS_DISTRO})

ament_target_dependencies(${PROJECT_NAME}
                         angles
                         Eigen3
                         geometry_msgs
                         image_geometry
                         multisense_lib
                         multisense_msgs
                         OpenCV
                         rclcpp
                         sensor_msgs
                         stereo_msgs
                         tf2
                         tf2_ros
                         tf2_geometry_msgs)


add_executable(ros_driver src/ros_driver.cpp)
target_link_libraries(ros_driver ${PROJECT_NAME})

ament_target_dependencies(ros_driver
                         rclcpp)

install(TARGETS ros_driver
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch materials meshes urdf
        DESTINATION share/${PROJECT_NAME})

ament_package()
