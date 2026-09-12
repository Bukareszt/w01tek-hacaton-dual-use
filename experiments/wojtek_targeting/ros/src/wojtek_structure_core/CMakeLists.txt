cmake_minimum_required(VERSION 3.8)
project(wojtek_structure_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# The Structure SDK is closed-source and cannot be vendored into a public
# repository, so it is located at configure time instead. Point the build at an
# unpacked SDK with -DSTRUCTURE_SDK_DIR=... or the matching environment
# variable. Both the SDK's own layout (Libraries/Structure/...) and a plain
# include/lib install are searched.
find_path(
  STRUCTURE_INCLUDE_DIR
  NAMES ST/CaptureSession.h
  HINTS ${STRUCTURE_SDK_DIR} $ENV{STRUCTURE_SDK_DIR}
  PATH_SUFFIXES Libraries/Structure/Headers Headers include)

find_library(
  STRUCTURE_LIBRARY
  NAMES Structure
  HINTS ${STRUCTURE_SDK_DIR} $ENV{STRUCTURE_SDK_DIR}
  PATH_SUFFIXES Libraries/Structure/Linux-arm64 Libraries/Structure/Linux-x86_64 lib)

if(NOT STRUCTURE_INCLUDE_DIR OR NOT STRUCTURE_LIBRARY)
  message(
    FATAL_ERROR
      "Structure SDK not found. Download the Cross-Platform SDK from the "
      "Structure Developer Portal, unpack it, and re-run with "
      "-DSTRUCTURE_SDK_DIR=/path/to/StructureSDK-CrossPlatform "
      "(or export STRUCTURE_SDK_DIR). See this package's README.")
endif()

get_filename_component(STRUCTURE_LIBRARY_DIR ${STRUCTURE_LIBRARY} DIRECTORY)

message(STATUS "Structure SDK headers: ${STRUCTURE_INCLUDE_DIR}")
message(STATUS "Structure SDK library: ${STRUCTURE_LIBRARY}")

add_executable(structure_core_node src/structure_core_node.cpp)

target_include_directories(structure_core_node PRIVATE ${STRUCTURE_INCLUDE_DIR})
target_link_libraries(structure_core_node ${STRUCTURE_LIBRARY})
ament_target_dependencies(structure_core_node rclcpp sensor_msgs)

# libStructure.so is not on the system loader path; it ships beside the SDK.
set_target_properties(
  structure_core_node PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE INSTALL_RPATH
                                 "${STRUCTURE_LIBRARY_DIR}")

install(TARGETS structure_core_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
