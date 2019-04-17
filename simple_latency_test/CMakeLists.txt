cmake_minimum_required(VERSION 3.5)
project(simple_latency_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


set(dependencies
  rclcpp
)

rosidl_generate_interfaces(${PROJECT_NAME}
  interfaces/msg/Dynamic.msg
  interfaces/msg/Array250kb.msg
  interfaces/msg/Array750kb.msg
  interfaces/msg/Array1mb.msg
  DEPENDENCIES std_msgs
)

add_executable(pub_sub_dynamic src/pub_sub_dynamic.cpp)
ament_target_dependencies(pub_sub_dynamic ${dependencies})
rosidl_target_interfaces(pub_sub_dynamic ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(pub_sub_250kb src/pub_sub_250kb.cpp)
ament_target_dependencies(pub_sub_250kb ${dependencies})
rosidl_target_interfaces(pub_sub_250kb ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(pub_sub_750kb src/pub_sub_750kb.cpp)
ament_target_dependencies(pub_sub_750kb ${dependencies})
rosidl_target_interfaces(pub_sub_750kb ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(pub_sub_1mb src/pub_sub_1mb.cpp)
ament_target_dependencies(pub_sub_1mb ${dependencies})
rosidl_target_interfaces(pub_sub_1mb ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(TARGETS
  pub_sub_dynamic
  pub_sub_250kb
  pub_sub_750kb
  pub_sub_1mb
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()