#include <memory>
#include <vector>
#include <string>
#include <chrono>

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


  move_group_interface.setMaxVelocityScalingFactor(0.1);      //  max hitrosti
  move_group_interface.setMaxAccelerationScalingFactor(0.1);  //  max pospeška


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
         "manipulator")](auto const trajectory) {
      moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
    };
*/
  // Set a target Pose
  struct Target {
    geometry_msgs::msg::Pose pose;
    double velocity_scaling;
    double acceleration_scaling;
    double delay_seconds;
    double vacuum_action;
  };

  std::vector<Target> targets;

  std::vector<double> target_values;
  node->get_parameter("targets", target_values);

  constexpr size_t kTargetFieldCount = 10;
  if (target_values.empty() || target_values.size() % kTargetFieldCount != 0) {
    RCLCPP_ERROR(logger, "Parameter 'targets' mora imeti skupine po 10 vrednosti.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  for (size_t i = 0; i < target_values.size(); i += kTargetFieldCount) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = target_values[i + 0];
    pose.position.y = target_values[i + 1];
    pose.position.z = target_values[i + 2];

    tf2::Quaternion q;
    q.setRPY(target_values[i + 3], target_values[i + 4], target_values[i + 5]);
    q.normalize();
    pose.orientation = tf2::toMsg(q);

    double velocity_scaling = target_values[i + 6];
    double acceleration_scaling = target_values[i + 7];
    double delay_seconds = target_values[i + 8];
    double vacuum_action = target_values[i + 9];


    targets.push_back({pose, velocity_scaling, acceleration_scaling, delay_seconds, vacuum_action});
  }

  for (size_t i = 0; i < targets.size(); ++i) {
    const auto & target = targets[i];

    Eigen::Isometry3d point_pose = Eigen::Isometry3d::Identity();
    point_pose.translation().x() = target.pose.position.x;
    point_pose.translation().y() = target.pose.position.y;
    point_pose.translation().z() = target.pose.position.z;

    moveit_visual_tools.publishSphere(
      point_pose,
      rviz_visual_tools::RED,
      rviz_visual_tools::LARGE
    );
/*
    Eigen::Isometry3d text_pose = point_pose;
    text_pose.translation().z() += 0.08;

    moveit_visual_tools.publishText(
      text_pose,
      std::string("T") + std::to_string(i + 1),
      rviz_visual_tools::WHITE,
      rviz_visual_tools::XLARGE
    );*/
  }

  moveit_visual_tools.trigger();


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
    const auto & target = targets[i];

    move_group_interface.setMaxVelocityScalingFactor(target.velocity_scaling);
    move_group_interface.setMaxAccelerationScalingFactor(target.acceleration_scaling);

    move_group_interface.setStartStateToCurrentState();
    move_group_interface.setPoseTarget(target.pose);

    auto const [success, plan] = [&move_group_interface]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
      //draw_trajectory_tool_path(plan.trajectory);
      moveit_visual_tools.trigger();
      prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
      draw_title("Executing");
      moveit_visual_tools.trigger();
      move_group_interface.execute(plan);
      move_group_interface.clearPoseTargets();
      if (target.vacuum_action == 1.0) {
        RCLCPP_INFO(logger, "Vklop vakuuma"); // vklopi vakuum
      }
      else if (target.vacuum_action == 2.0) {
        RCLCPP_INFO(logger, "Izklop vakuuma"); // izklopi vakuum
      }
      if (target.delay_seconds > 0.0) {
        rclcpp::sleep_for(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(target.delay_seconds)
          )
        );
      }

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