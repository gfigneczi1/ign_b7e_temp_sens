cmake_minimum_required(VERSION 3.8)
project(temp_sens)

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
find_package(sensor_msgs REQUIRED)

set(ament_dependencies
  rclcpp
  std_msgs
  sensor_msgs
)

add_executable(sensor_node src/sens_emulator.cpp)
add_executable(sensor_monitor src/sens_monitor.cpp)
ament_target_dependencies(sensor_node ${ament_dependencies} )
ament_target_dependencies(sensor_monitor ${ament_dependencies} )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
sensor_node
sensor_monitor
DESTINATION lib/${PROJECT_NAME})

ament_package()