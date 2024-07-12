cmake_minimum_required(VERSION 3.8)
project(robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(control_msgs REQUIRED) # Ensure control_msgs is found

# Generate the executable
add_executable(move_robot src/move_robot.cpp)
target_include_directories(move_robot PUBLIC include)
ament_target_dependencies(move_robot
  ament_cmake
  sensor_msgs
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
  control_msgs) 

add_executable(move_robot_targetpose src/move_robot_targetpose.cpp)
target_include_directories(move_robot_targetpose PUBLIC include)
ament_target_dependencies(move_robot_targetpose
  ament_cmake
  sensor_msgs
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
  control_msgs) 
# Install the executable
install(TARGETS move_robot move_robot_targetpose
  DESTINATION lib/${PROJECT_NAME})

# Install the launch file
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
