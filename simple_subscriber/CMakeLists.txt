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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
)

include_directories(include)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/simple_subscriber_node.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(subscriber_main
  src/subscriber_main.cpp
)

target_link_libraries(subscriber_main
  ${library_name}
)

ament_target_dependencies(subscriber_main
  ${dependencies}
)

install(TARGETS subscriber_main ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
