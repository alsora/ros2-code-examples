cmake_minimum_required(VERSION 3.5)
project(logger_verbosity)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(logger_main main.cpp)

ament_target_dependencies(logger_main rclcpp)

install(TARGETS logger_main
  DESTINATION lib/${PROJECT_NAME})

ament_package()
