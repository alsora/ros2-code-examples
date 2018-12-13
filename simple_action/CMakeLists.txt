cmake_minimum_required(VERSION 3.5)
project(simple_action)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(simple_interfaces REQUIRED)

#include_directories(include)

add_executable(client_main src/client_main.cpp)
ament_target_dependencies(client_main rclcpp rclcpp_action simple_interfaces)

add_executable(service_main src/service_main.cpp)
ament_target_dependencies(service_main rclcpp rclcpp_action simple_interfaces)


install(TARGETS client_main service_main
  DESTINATION lib/${PROJECT_NAME})

ament_package()
