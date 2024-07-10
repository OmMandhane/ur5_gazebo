# #!/usr/bin/env python3

# import sys
# import rclpy
# from geometry_msgs.msg import Pose, Point, Quaternion
# from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander

# def main():
#     rclpy.init(args=sys.argv)
#     node = rclpy.create_node('moveit_planning_execution')

#     robot = RobotCommander()
#     group_name = 'manipulator'
#     move_group = MoveGroupCommander(group_name)

#     # Set a target pose
#     target_pose = Pose()
#     target_pose.position = Point(x=0.5, y=0.0, z=0.5)  # Example position
#     target_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # Example orientation

#     # Set the target pose
#     move_group.set_pose_target(target_pose)

#     # Plan the motion
#     plan = move_group.go(wait=True)
#     if not plan:
#         node.get_logger().error('Failed to plan motion')
#     else:
#         node.get_logger().info('Motion planning succeeded')

#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

