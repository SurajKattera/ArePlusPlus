cmake_minimum_required(VERSION 3.5)
project(areplusplus)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(apriltag_msgs REQUIRED)

add_executable(TaskHandler src/main.cpp src/Structures.h 
src/TaskPlanner.h src/TaskPlanner.cpp src/moving_node.h 
src/moving_node.cpp src/artag_detector_node.cpp src/artag_detector_node.h)

ament_target_dependencies(TaskHandler rclcpp nav2_msgs 
rclcpp_action tf2 nav_msgs std_msgs sensor_msgs geometry_msgs 
OpenCV cv_bridge apriltag_msgs)

install(TARGETS
  TaskHandler
  DESTINATION lib/${PROJECT_NAME}/
  )

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
  )
install(FILES
  config/tasks.txt
  config/locations.txt
  DESTINATION share/${PROJECT_NAME}/config/
  )
  
ament_package()
