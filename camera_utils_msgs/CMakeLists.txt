cmake_minimum_required(VERSION 3.5)
project(camera_utils_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
)

find_package(message_generation REQUIRED)

add_message_files(
  FILES
  Frames.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)


