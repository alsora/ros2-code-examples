cmake_minimum_required(VERSION 3.5)
project(simple_client)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(client_main client_main.cpp)
ament_target_dependencies(client_main rclcpp my_interfaces sensor_msgs)

add_executable(asynchronous_main asynchronous_main.cpp)
ament_target_dependencies(asynchronous_main rclcpp my_interfaces sensor_msgs)

install(TARGETS client_main asynchronous_main
  DESTINATION lib/${PROJECT_NAME})

ament_package()
