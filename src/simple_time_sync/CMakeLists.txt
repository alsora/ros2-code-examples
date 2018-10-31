cmake_minimum_required(VERSION 3.5)
project(simple_time_sync)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_interfaces REQUIRED)
find_package(message_filters REQUIRED)

add_executable(time_sync_main main.cpp)
add_executable(method method.cpp)


ament_target_dependencies(time_sync_main rclcpp std_msgs my_interfaces message_filters)
ament_target_dependencies(method rclcpp std_msgs my_interfaces message_filters)

install(TARGETS time_sync_main method
  DESTINATION lib/${PROJECT_NAME})

ament_package()
