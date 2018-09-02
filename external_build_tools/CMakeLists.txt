cmake_minimum_required(VERSION 3.5)
project(ros2_cmake_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#MESSAGE( STATUS "INCLUDE is: " ${rclcpp_INCLUDE_DIRS} )
#MESSAGE( STATUS "LIB is: " ${rclcpp_LIBRARIES} )

MESSAGE( STATUS "LIB is: " ${std_msgs_LIBRARIES} )

include_directories(${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
add_executable(hello_ros hello_ros.cpp)
target_link_libraries(hello_ros ${rclcpp_LIBRARIES} )

install(TARGETS hello_ros
  DESTINATION bin)


