cmake_minimum_required(VERSION 3.5)
project(simple_multithread)

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


add_executable(simple_pub_sub_std simple_pub_sub_std.cpp multi-node.cpp)
ament_target_dependencies(simple_pub_sub_std rclcpp std_msgs std_srvs)

add_executable(simple_pub_sub_pthread simple_pub_sub_pthread.cpp multi-node.cpp)
ament_target_dependencies(simple_pub_sub_pthread rclcpp std_msgs std_srvs)

add_executable(simple_client_service_pthread simple_client_service_pthread.cpp multi-node.cpp)
ament_target_dependencies(simple_client_service_pthread rclcpp std_msgs std_srvs)

install(TARGETS
  simple_pub_sub_std 
  simple_pub_sub_pthread 
  simple_client_service_pthread
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
