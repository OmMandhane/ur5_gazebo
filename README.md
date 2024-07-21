This repo has a moveit planner to control ur5 robot in gazebo simulation using trac_ik as the inverse kinematic solver for ros2 humble.
Steps to use this:
  1. Clone this repo and delete the log, install and build files.
  2. 
  3. Open terminal and colcon build the file
  Now buid the workspace again and run this command ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py.
 This will launch the gazebo sim with moveit planner in rviz.
         
