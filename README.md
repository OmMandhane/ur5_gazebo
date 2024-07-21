# UR5 Controller using trac_ik and MoveIt

## Preview:
This repo has a MoveIt planner to control UR5 robot in Gazebo simulation using trac_ik as the inverse kinematics solver for ROS 2 humble in Ubuntu 22.04.
     
## Installation:
  1. Clone this repo and delete the log, install and build files.
  2. Go into the cloned directory using `cd ur5_gazebo` and then enter the following commands to install the necessary packages using rosdep.
     ```
     sudo apt install python3-rosdep
     sudo rosdep init
     rosdep update
     rosdep install --from-paths src --ignore-src -r -y
     ```
  3. Now use `colcon build` command to build the file.

     (**NOTE** : If colcon is not already installed in your system, install it using -
     ```
     sudo apt install python3-colcon-common-extensions
     ```
     
  4. Source the workspace using -
     ```
     source install/setup.bash
     ```

## Usage:
  1. Launch the RViz Gazebo and MoveIt using command -
     ```
     ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
     ```
     
  2. You can now move the end effector to desired coordinate and orientation by adding the coordinate and quaternion angle for orientation. Below is the          sample command -
     ```
      ros2 launch robot_controller move_robot_targetpose.launch.py tp_x:=0.4 tp_y:=0.1 tp_z:=0.8 to_x:=0.0 to_y:=0.0 to_z:=0.0 to_w:=1.0 
     ```

     **NOTE** : You can change the coordinates by changing tp_x, tp_y and tp_z. For orientation, change to_x, to_y, to_z and to_w      
