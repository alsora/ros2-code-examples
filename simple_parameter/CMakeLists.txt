cmake_minimum_required(VERSION 3.5)
project(simple_parameter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(dependencies
  rclcpp
)

include_directories(include)

add_executable(parameter_server_main
  src/parameter_server_main.cpp
  src/simple_parameter_server_node.cpp
)

ament_target_dependencies(parameter_server_main
  ${dependencies}
)

add_executable(parameter_client_main
  src/parameter_client_main.cpp
  src/simple_parameter_client_node.cpp
)

ament_target_dependencies(parameter_client_main
  ${dependencies}
)

install(TARGETS parameter_server_main parameter_client_main
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
