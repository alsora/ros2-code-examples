cmake_minimum_required(VERSION 3.5)
project(simple_static_executor)

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


add_executable(static_executor_test static_executor_test.cpp)
ament_target_dependencies(static_executor_test rclcpp std_msgs)

install(TARGETS static_executor_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
