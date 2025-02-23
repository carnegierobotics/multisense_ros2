cmake_minimum_required(VERSION 3.15)
project(multisense_lib)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Selecting default build type: Release")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default: Release)" FORCE)
endif ()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nlohmann_json 3.9 REQUIRED)

set(BUILD_LEGACY_API OFF CACHE BOOL "Build the new API" FORCE)
set(BUILD_OPENCV ON CACHE BOOL "Build with OpenCV" FORCE)
set(BUILD_JSON_SERIALIZATION ON CACHE BOOL "Build with JSON" FORCE)
add_subdirectory(LibMultiSense)

ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/LibMultiSense/source/LibMultiSense/include)
ament_export_libraries(MultiSense)

ament_package()
