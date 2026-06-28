#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>  // <---- add this to the set of includes at the top
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "hello_moveit/collision_objects.hpp"


int main(int argc, char ** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "fanuc_arm");
  move_group_interface.setEndEffectorLink("tool_tip");


  move_group_interface.setMaxVelocityScalingFactor(0.05);      // 20 % max hitrosti
  move_group_interface.setMaxAccelerationScalingFactor(0.05);  // 20 % max pospeška



  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create closures for visualization
auto const draw_title = [&moveit_visual_tools](auto text) {
  auto const text_pose = [] {
    auto msg = Eigen::Isometry3d::Identity();
    msg.translation().z() = 1.8;  // Place text 1m above the base link
    return msg;
  }();
  moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                  rviz_visual_tools::XLARGE);
};
auto const prompt = [&moveit_visual_tools](auto text) {
  moveit_visual_tools.prompt(text);
};
/*auto const draw_trajectory_tool_path =
    [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup(
         "hand")](auto const trajectory) {
      moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
    };
*/

auto const current_tcp_pose = move_group_interface.getCurrentPose("tool_tip").pose;
auto const fixed_position = current_tcp_pose.position;

  // Set a target Pose
  std::vector<geometry_msgs::msg::Pose> targets;

  geometry_msgs::msg::Pose pose1;
  pose1.position = fixed_position;
  tf2::Quaternion q1;
  q1.setRPY(-0.5, 0.0, 0.0);  // roll, pitch, yaw
  pose1.orientation = tf2::toMsg(q1);
  targets.push_back(pose1);

  geometry_msgs::msg::Pose pose2;
  pose2.position = fixed_position;
  tf2::Quaternion q2;
  q2.setRPY(0.0, 0.0, 0.0);  // roll, pitch, yaw
  pose2.orientation = tf2::toMsg(q2);
  targets.push_back(pose2);

  geometry_msgs::msg::Pose pose3;
  pose3.position = fixed_position;
  tf2::Quaternion q3;
  q3.setRPY(0.5, 0.0, 0.0);  // roll, pitch, yaw
  pose3.orientation = tf2::toMsg(q3);
  targets.push_back(pose3);

  targets.push_back(pose2);


  // Create collision object for the robot to avoid
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  auto collision_objects =
      hello_moveit::makeCollisionObjects(move_group_interface.getPlanningFrame());

  planning_scene_interface.applyCollisionObjects(collision_objects);

  /*
  // Create a plan to that target pose
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  */

  for (size_t i = 0; i < targets.size(); ++i) {
    const auto & target_pose = targets[i];

    move_group_interface.setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction = move_group_interface.computeCartesianPath(
        waypoints,
        0.005,   // eef_step
        trajectory
    );

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory = trajectory;

  bool success = fraction > 0.95;

  RCLCPP_INFO(logger, "Cartesian path fraction: %.2f", fraction);


    // Execute the plan
    if(success) {
      //draw_trajectory_tool_path(plan.trajectory);
      moveit_visual_tools.trigger();
      prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
      draw_title("Executing");
      moveit_visual_tools.trigger();
      move_group_interface.execute(plan);
      move_group_interface.clearPoseTargets();
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
    } else {
      draw_title("Planning Failed!");
      moveit_visual_tools.trigger();
      RCLCPP_ERROR(logger, "Planning failed!");
      break;
    }
  }
  
  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}