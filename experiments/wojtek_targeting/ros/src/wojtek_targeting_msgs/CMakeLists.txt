cmake_minimum_required(VERSION 3.8)
project(wojtek_targeting_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Interfaces only. The nodes live in wojtek_targeting, which is ament_python --
# keeping the generator in its own package is what lets that one stay Python.
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/LaserTarget.msg
  msg/TargetingStatus.msg
  srv/SetGimbalAngles.srv
  DEPENDENCIES std_msgs)

ament_package()
