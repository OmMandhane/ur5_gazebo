This repo has a moveit planner to control ur5 robot in gazebo simulation using trac_ik as the inverse kinematic solver for ros2 humble.
Steps to use this:
  1. Clone this repo and delete the log, install and build files.
  2. Go into the main file using "cd ur5_gazebo"  and then enter the following commands to install the necessary packages using rosdep
  3. sudo apt install python3-rosdep
  4. sudo rosdep init
  5. rosdep update
  6. rosdep install --from-paths src --ignore-src -r -y
  7. Now use "colcon build" command to build the file
  8. Source the workspace using source install/setup.bash
  9. now launch the rviz gazebo and moveit using this command "ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py"
  10. You can now move the end effector to disired coordinate and orientation by adding the coordinate and quaternion angle for orientation in this command
      "ros2 launch robot_controller move_robot_targetpose.launch.py tp_x:=0.4 tp_y:=0.1 tp_z:=0.8 to_x:=0.0 to_y:=0.0 to_z:=0.0 to_w:=1.0" 
 

         
