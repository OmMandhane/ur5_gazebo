#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/joint_state.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP_ARM = "ur_manipulator";

class TestTrajectory : public rclcpp::Node {
public:
  TestTrajectory(std::shared_ptr<rclcpp::Node> move_group_node)
      : Node("test_trajectory"),
        move_group_arm(move_group_node, PLANNING_GROUP_ARM),
        joint_model_group_arm(
            move_group_arm.getCurrentState()->getJointModelGroup(
                PLANNING_GROUP_ARM)) {
    

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&TestTrajectory::joint_state_callback, this, std::placeholders::_1));

    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&TestTrajectory::timer_callback, this));

    this->declare_parameter("tp.x", -0.5);
    this->declare_parameter("tp.y", 0.0);
    this->declare_parameter("tp.z", 0.5);
    this->declare_parameter("to.x", 0.0);
    this->declare_parameter("to.y", 0.0);
    this->declare_parameter("to.z", 0.0);
    this->declare_parameter("to.w", 1.0);
  }

  void get_info() {
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_arm.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End-effector link: %s", move_group_arm.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group_arm.getJointModelGroupNames().begin(), move_group_arm.getJointModelGroupNames().end(),
                std::ostream_iterator<std::string>(std::cout, ", "));
  }

  void current_state() {
    RCLCPP_INFO(LOGGER, "Get Robot Current State");
    current_state_arm = move_group_arm.getCurrentState(10);
    if (!current_state_arm) {
      RCLCPP_ERROR(LOGGER, "Failed to get current state");
      return;
    }
    current_state_arm->copyJointGroupPositions(this->joint_model_group_arm,
                                               this->joint_group_positions_arm);
  }

  void plan_arm_target_pose() {
    RCLCPP_INFO(LOGGER, "Planning to Target Pose");

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = this->get_parameter("tp.x").as_double();
    target_pose.position.y = this->get_parameter("tp.y").as_double();
    target_pose.position.z = this->get_parameter("tp.z").as_double();
    target_pose.orientation.x = this->get_parameter("to.x").as_double();
    target_pose.orientation.y = this->get_parameter("to.y").as_double();
    target_pose.orientation.z = this->get_parameter("to.z").as_double();
    target_pose.orientation.w = this->get_parameter("to.w").as_double();


    move_group_arm.setPoseTarget(target_pose);

    // Plan to the target pose
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    bool success_arm = (move_group_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success_arm) {
        RCLCPP_ERROR(LOGGER, "Planning failed");
        return;
    }

    // Execute the plan
    moveit::planning_interface::MoveItErrorCode execute_result = move_group_arm.execute(my_plan_arm);
    if (execute_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Execution failed");
    }
  }

    void add_ground_plane() {
    moveit_msgs::msg::CollisionObject ground_plane;
    ground_plane.id = "ground_plane";
    ground_plane.header.frame_id = move_group_arm.getPlanningFrame();

    shape_msgs::msg::SolidPrimitive plane;
    plane.type = plane.BOX;
    plane.dimensions = {5.0, 5.0, 0.01}; // A large flat plane

    geometry_msgs::msg::Pose plane_pose;
    plane_pose.orientation.w = 1.0;
    plane_pose.position.z = -0.005; // Position the plane slightly below the robot base

    ground_plane.primitives.push_back(plane);
    ground_plane.primitive_poses.push_back(plane_pose);
    ground_plane.operation = ground_plane.ADD;

    planning_scene_interface.applyCollisionObject(ground_plane);
  }

  void timer_callback() {
    if (!move_group_arm.getCurrentState()) {
      RCLCPP_WARN(LOGGER, "Failed to get current state. Retrying in 1 second.");
      this->timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                             std::bind(&TestTrajectory::timer_callback, this));
      return;
    }
    this->timer_->cancel();
    add_ground_plane();
    get_info();
    current_state();
    plan_arm_target_pose();
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    RCLCPP_INFO(LOGGER, "Received joint states. First joint position: %f", msg->position[0]);
  }

private:
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<double> joint_group_positions_arm;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  rclcpp::TimerBase::SharedPtr timer_;
  moveit::planning_interface::MoveGroupInterface move_group_arm;
  const moveit::core::JointModelGroup *joint_model_group_arm;
  moveit::core::RobotStatePtr current_state_arm;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_demo", node_options);
  rclcpp::executors::SingleThreadedExecutor planner_executor;
  std::shared_ptr<TestTrajectory> planner_node = std::make_shared<TestTrajectory>(move_group_node);
  planner_executor.add_node(planner_node);
  planner_executor.spin();

  rclcpp::shutdown();
  return 0;
}