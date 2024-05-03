cmake_minimum_required(VERSION 3.5)
project(ccp_js_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)  # Updated from C++14 to C++17
endif()

# Ensure the compiler uses C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)


add_executable(ccp_js_node src/ccp_js.cpp)
ament_target_dependencies(ccp_js_node rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs tf2_ros tf2 tf2_geometry_msgs ament_index_cpp)

ament_target_dependencies(ccp_js_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  ament_index_cpp
)

install(TARGETS
  ccp_js_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  include/
  DESTINATION include
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
