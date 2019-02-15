cmake_minimum_required(VERSION 3.5)
project(simple_memory_test)

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


set(dependencies
  rclcpp
)

rosidl_generate_interfaces(${PROJECT_NAME}
  interfaces/msg/Array10b.msg
  interfaces/msg/Array1kb.msg
  interfaces/msg/Array5kb.msg
  interfaces/msg/Array10kb.msg
  interfaces/msg/Array50kb.msg
  interfaces/msg/Array100kb.msg
  interfaces/msg/Array250kb.msg
  interfaces/msg/Array300kb.msg
  interfaces/msg/Array400kb.msg
  interfaces/msg/Array450kb.msg
  interfaces/msg/Array500kb.msg
  interfaces/msg/Array750kb.msg
  interfaces/msg/Array1mb.msg
  interfaces/msg/Array2mb.msg
  interfaces/msg/Array4mb.msg
)

add_executable(sub_node_10b src/sub/10b_main.cpp)
ament_target_dependencies(sub_node_10b ${dependencies})
rosidl_target_interfaces(sub_node_10b ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sub_node_1kb src/sub/1kb_main.cpp)
ament_target_dependencies(sub_node_1kb ${dependencies})
rosidl_target_interfaces(sub_node_1kb ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sub_node_5kb src/sub/5kb_main.cpp)
ament_target_dependencies(sub_node_5kb ${dependencies})
rosidl_target_interfaces(sub_node_5kb ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sub_node_10kb src/sub/10kb_main.cpp)
ament_target_dependencies(sub_node_10kb ${dependencies})
rosidl_target_interfaces(sub_node_10kb ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sub_node_50kb src/sub/50kb_main.cpp)
ament_target_dependencies(sub_node_50kb ${dependencies})
rosidl_target_interfaces(sub_node_50kb ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sub_node_100kb src/sub/100kb_main.cpp)
ament_target_dependencies(sub_node_100kb ${dependencies})
rosidl_target_interfaces(sub_node_100kb ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sub_node_250kb src/sub/250kb_main.cpp)
ament_target_dependencies(sub_node_250kb ${dependencies})
rosidl_target_interfaces(sub_node_250kb ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sub_node_300kb src/sub/300kb_main.cpp)
ament_target_dependencies(sub_node_300kb ${dependencies})
rosidl_target_interfaces(sub_node_300kb ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sub_node_400kb src/sub/400kb_main.cpp)
ament_target_dependencies(sub_node_400kb ${dependencies})
rosidl_target_interfaces(sub_node_400kb ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sub_node_450kb src/sub/450kb_main.cpp)
ament_target_dependencies(sub_node_450kb ${dependencies})
rosidl_target_interfaces(sub_node_450kb ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sub_node_500kb src/sub/500kb_main.cpp)
ament_target_dependencies(sub_node_500kb ${dependencies})
rosidl_target_interfaces(sub_node_500kb ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sub_node_750kb src/sub/750kb_main.cpp)
ament_target_dependencies(sub_node_750kb ${dependencies})
rosidl_target_interfaces(sub_node_750kb ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sub_node_1mb src/sub/1mb_main.cpp)
ament_target_dependencies(sub_node_1mb ${dependencies})
rosidl_target_interfaces(sub_node_1mb ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sub_node_2mb src/sub/2mb_main.cpp)
ament_target_dependencies(sub_node_2mb ${dependencies})
rosidl_target_interfaces(sub_node_2mb ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sub_node_4mb src/sub/4mb_main.cpp)
ament_target_dependencies(sub_node_4mb ${dependencies})
rosidl_target_interfaces(sub_node_4mb ${PROJECT_NAME} "rosidl_typesupport_cpp")



add_executable(pub_node_10b src/pub/10b_main.cpp)
ament_target_dependencies(pub_node_10b ${dependencies})
rosidl_target_interfaces(pub_node_10b ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(pub_node_100kb src/pub/100kb_main.cpp)
ament_target_dependencies(pub_node_100kb ${dependencies})
rosidl_target_interfaces(pub_node_100kb ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(pub_node_500kb src/pub/500kb_main.cpp)
ament_target_dependencies(pub_node_500kb ${dependencies})
rosidl_target_interfaces(pub_node_500kb ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(pub_node_4mb src/pub/4mb_main.cpp)
ament_target_dependencies(pub_node_4mb ${dependencies})
rosidl_target_interfaces(pub_node_4mb ${PROJECT_NAME} "rosidl_typesupport_cpp")



install(TARGETS
    sub_node_10b sub_node_1kb sub_node_5kb sub_node_10kb sub_node_50kb
    sub_node_100kb sub_node_250kb sub_node_300kb sub_node_400kb sub_node_450kb sub_node_500kb sub_node_750kb
    sub_node_1mb sub_node_2mb sub_node_4mb
    pub_node_10b pub_node_100kb pub_node_500kb pub_node_4mb
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()