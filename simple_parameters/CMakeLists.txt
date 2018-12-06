cmake_minimum_required(VERSION 3.5)
project(simple_parameters)

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

add_executable(parameters_server_main
  src/parameters_server_main.cpp
  src/simple_parameters_server_node.cpp
)

ament_target_dependencies(parameters_server_main
  ${dependencies}
)

add_executable(parameters_client_main
  src/parameters_client_main.cpp
  src/simple_parameters_client_node.cpp
)

ament_target_dependencies(parameters_client_main
  ${dependencies}
)

install(TARGETS parameters_server_main parameters_client_main
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
