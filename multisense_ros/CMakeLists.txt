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
generate_parameter_library(init_parameters src/init_parameters.yaml)

add_library(${PROJECT_NAME} src/multisense.cpp)

target_link_libraries(${PROJECT_NAME} init_parameters multisense_parameters)

ament_target_dependencies(${PROJECT_NAME}
                         angles
                         Eigen3
                         geometry_msgs
                         image_geometry
                         image_transport
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

ament_export_targets(${PROJECT_NAME} multisense_parameters init_parameters)

ament_export_dependencies(
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
    tf2_geometry_msgs
)

install(TARGETS ros_driver
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch materials meshes urdf
        DESTINATION share/${PROJECT_NAME})

install(TARGETS multisense_parameters
        EXPORT multisense_parameters
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
)

install(TARGETS init_parameters
        EXPORT init_parameters
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
)

ament_package()
