cmake_minimum_required(VERSION 3.5)
project(simple_backward_compatible)

## Comment the following line if you want to build V1 of the code to test backward compatibility
#set(BUILD_V2 true)

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

if (BUILD_V2)
  add_definitions(-DBUILD_V2)
  set (INTERFACES_PATH "interfaces_v2")
else()
  set (INTERFACES_PATH "interfaces_v1")
endif()

set(dependencies
  rclcpp
)

message(" -------------------> BUILDING ${INTERFACES_PATH}")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${INTERFACES_PATH}/msg/OptionalFields.msg
)

add_executable(publisher_main src/publisher_main.cpp)
ament_target_dependencies(publisher_main ${dependencies})
rosidl_target_interfaces(publisher_main ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(subscriber_main src/subscriber_main.cpp)
ament_target_dependencies(subscriber_main ${dependencies})
rosidl_target_interfaces(subscriber_main ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS publisher_main subscriber_main
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()