cmake_minimum_required(VERSION 3.5)
project(simple_security)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(simple_interfaces REQUIRED)


add_executable(publishers_main publishers_main.cpp)
ament_target_dependencies(publishers_main rclcpp simple_interfaces)

add_executable(subscribers_main subscribers_main.cpp)
ament_target_dependencies(subscribers_main rclcpp simple_interfaces)

install(TARGETS publishers_main subscribers_main
  DESTINATION lib/${PROJECT_NAME})

ament_package()
