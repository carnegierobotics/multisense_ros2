cmake_minimum_required(VERSION 3.15)
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
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(multisense_lib REQUIRED)
find_package(MultiSense REQUIRED)
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

generate_parameter_library(multisense_parameters src/multisense_parameters.yaml)
generate_parameter_library(multisense_init_parameters src/init_parameters.yaml)

add_library(${PROJECT_NAME} src/multisense.cpp src/publisher_utilities.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC multisense_init_parameters multisense_parameters MultiSense)

target_include_directories(${PROJECT_NAME}
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PUBLIC $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
                         angles
                         Eigen3
                         geometry_msgs
                         generate_parameter_library
                         image_geometry
                         image_transport
                         MultiSense
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
target_link_libraries(ros_driver PUBLIC ${PROJECT_NAME})

ament_target_dependencies(ros_driver PUBLIC
                          rclcpp)

install(
  TARGETS
    ${PROJECT_NAME}
    multisense_init_parameters
    multisense_parameters
    ros_driver
  EXPORT multisense_ros-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS ros_driver
        DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include
)

install(
  DIRECTORY launch materials meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  EXPORT multisense_ros-targets
  NAMESPACE multisense_ros::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(multisense_ros-targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_export_dependencies(
  angles
  Eigen3
  geometry_msgs
  generate_parameter_library
  image_geometry
  image_transport
  MultiSense
  multisense_lib
  multisense_msgs
  OpenCV
  rclcpp
  sensor_msgs
  stereo_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_package()
