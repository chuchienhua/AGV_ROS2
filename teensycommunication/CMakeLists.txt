cmake_minimum_required(VERSION 3.5)
project(teensycommunication)

# Default to C99 for C files
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14 for C++ files
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)  # Ensure serial is found as a package

include_directories(
  include
  ${serial_INCLUDE_DIRS}  # Add the include directory for serial
)

# Declare a C++ executable
add_executable(teensycommunication
  src/teensycommunication_node.cpp
  src/teensycommunication.cpp
)
  # Make sure the path to your source file is correct

# Specify libraries or flags to use when linking the target
ament_target_dependencies(teensycommunication
  rclcpp
  std_msgs
  serial  # Add serial to the target dependencies
)

# Install the executable
install(TARGETS
  teensycommunication
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

