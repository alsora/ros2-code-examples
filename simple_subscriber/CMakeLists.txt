cmake_minimum_required(VERSION 3.5)
project(simple_subscriber)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} simple_subscriber.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

add_executable(subscriber_main subscriber_main.cpp simple_subscriber.cpp)
ament_target_dependencies(subscriber_main rclcpp std_msgs)

install(TARGETS subscriber_main
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)

ament_package()
