cmake_minimum_required(VERSION 3.5)
project(multithread)

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
find_package(std_srvs REQUIRED)

add_executable(main main.cpp)
ament_target_dependencies(main rclcpp std_msgs std_srvs)

add_executable(intra_process intra_process.cpp)
ament_target_dependencies(intra_process rclcpp std_msgs std_srvs)

install(TARGETS main intra_process
  DESTINATION lib/${PROJECT_NAME})

ament_package()
