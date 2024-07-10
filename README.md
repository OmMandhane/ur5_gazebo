This repo has a moveit planner to control ur5 robot in gazebo simulation using trac_ik as the inverse kinematic solver for ros2 humble.
Steps to use this:
  1. Clone this repo and delete the log, install and build files.
  2. Open terminal and colcon build the file. This will install the necessary files like ur_urdf and ur_moveit_config in your /opt/ros/humble/share/ folder.
  3. If a error is shown during building build the necessary files using rosdep
  4. Now to change the ik solver u need to find the ur_moveit_config file which would be most likely located in /opt/ros/humble/share/ur_moveit_config
  5. open this file and find the kinematics.yaml file created for your robot.
     Replace kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin (or similar) with kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
     you will have to use sudo administrative permissions to save the file.
  6. Now buid the workspace again and run this command ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py.
  7. This will launch the gazebo sim with moveit planner in rviz.
         
