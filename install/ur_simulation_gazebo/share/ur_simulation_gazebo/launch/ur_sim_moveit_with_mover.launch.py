from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Include the original ur_sim_moveit.launch.py
    ur_sim_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_simulation_gazebo"),
                'launch',
                'ur_sim_moveit.launch.py'
            ])
        ]),
    )

    # Define your C++ node
    robot_mover_node = Node(
        package='robot_controller',
        executable='move_robot',
        name='robot_mover',
        output='screen',
    )

    # Use TimerAction to delay the start of your node
    delayed_robot_mover = TimerAction(
        period=10.0,  # Delay in seconds
        actions=[robot_mover_node]
    )

    return LaunchDescription([
        ur_sim_moveit,
        delayed_robot_mover
    ])