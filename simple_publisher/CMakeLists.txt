cmake_minimum_required(VERSION 3.5)
project(simple_publisher)

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


add_executable(publisher_main publisher_main.cpp simple_publisher.cpp)
ament_target_dependencies(publisher_main rclcpp std_msgs)

install(TARGETS publisher_main
  DESTINATION lib/${PROJECT_NAME})

ament_package()
