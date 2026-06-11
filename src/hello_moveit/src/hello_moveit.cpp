#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
//#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>  // <---- add this to the set of includes at the top
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


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
/*
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
    msg.translation().z() = 1.0;  // Place text 1m above the base link
    return msg;
  }();
  moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                  rviz_visual_tools::XLARGE);
};
auto const prompt = [&moveit_visual_tools](auto text) {
  moveit_visual_tools.prompt(text);
};
auto const draw_trajectory_tool_path =
    [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup(
         "manipulator")](auto const trajectory) {
      moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
    };
*/
  // Set a target Pose
  std::vector<geometry_msgs::msg::Pose> targets;

  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 0.70;
  pose1.position.y = -0.40;
  pose1.position.z = 0.45;
  tf2::Quaternion q1;
  q1.setRPY(0.0, 0.0, 0.0);  // roll, pitch, yaw
  pose1.orientation = tf2::toMsg(q1);
  targets.push_back(pose1);

  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 0.60;
  pose2.position.y = 0.40;  
  pose2.position.z = 0.45;
  tf2::Quaternion q2;
  q2.setRPY(0.0, 0.4, 0.6);  // roll, pitch, yaw
  pose2.orientation = tf2::toMsg(q2);
  targets.push_back(pose2);

  geometry_msgs::msg::Pose pose3;
  pose3.position.x = 1.30;
  pose3.position.y = 0.20;
  pose3.position.z = 0.70;
  tf2::Quaternion q3;
  q3.setRPY(0.0, 1.2, 0.0);  // roll, pitch, yaw
  pose3.orientation = tf2::toMsg(q3);
  targets.push_back(pose3);


  // Create collision object for the robot to avoid
  auto const collision_object1 = [frame_id = move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    double box_x = 4.0; //Te spremenljivke ostanejo v temu bloku, če bi hotel da so globalne bi moral definirati izven bloka
    double box_y = 0.3;
    double box_z = 2.5;

    double box_pos_x = 1.1;
    double box_pos_y = 0.9 + (box_y / 2.0);
    double box_pos_z = box_z / 2.0;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = box_x;
    primitive.dimensions[primitive.BOX_Y] = box_y;
    primitive.dimensions[primitive.BOX_Z] = box_z;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
    box_pose.position.x = box_pos_x;
    box_pose.position.y = box_pos_y;
    box_pose.position.z = box_pos_z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  auto const collision_object2 = [frame_id = move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box2";
    shape_msgs::msg::SolidPrimitive primitive;

    double box_x = 0.3; //Te spremenljivke ostanejo v temu bloku, če bi hotel da so globalne bi moral definirati izven bloka
    double box_y = 0.5;
    double box_z = 2.5;

    double box_pos_x = -0.3 - (box_x / 2.0);
    double box_pos_y = 0.4 + (box_y / 2.0);
    double box_pos_z = box_z / 2.0;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = box_x;
    primitive.dimensions[primitive.BOX_Y] = box_y;
    primitive.dimensions[primitive.BOX_Z] = box_z;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
    box_pose.position.x = box_pos_x;
    box_pose.position.y = box_pos_y;
    box_pose.position.z = box_pos_z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  auto const collision_object3 = [frame_id = move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box3";
    shape_msgs::msg::SolidPrimitive primitive;

    double box_x = 0.3; //Te spremenljivke ostanejo v temu bloku, če bi hotel da so globalne bi moral definirati izven bloka
    double box_y = 2.2;
    double box_z = 2.5;

    double box_pos_x = -0.6 - (box_x / 2.0);
    double box_pos_y = -0.2;
    double box_pos_z = box_z / 2.0;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = box_x;
    primitive.dimensions[primitive.BOX_Y] = box_y;
    primitive.dimensions[primitive.BOX_Z] = box_z;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
    box_pose.position.x = box_pos_x;
    box_pose.position.y = box_pos_y;
    box_pose.position.z = box_pos_z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  auto const collision_object4 = [frame_id = move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box4";
    shape_msgs::msg::SolidPrimitive primitive;

    double box_x = 2.5; //Te spremenljivke ostanejo v temu bloku, če bi hotel da so globalne bi moral definirati izven bloka
    double box_y = 2.5;
    double box_z = 0.15;

    double box_pos_x = 0.35;
    double box_pos_y = -0.05;
    double box_pos_z = 2.35 + box_z / 2.0;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = box_x;
    primitive.dimensions[primitive.BOX_Y] = box_y;
    primitive.dimensions[primitive.BOX_Z] = box_z;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
    box_pose.position.x = box_pos_x;
    box_pose.position.y = box_pos_y;
    box_pose.position.z = box_pos_z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  auto const collision_object5 = [frame_id = move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "cylinder1";
    shape_msgs::msg::SolidPrimitive primitive;

    double cylinder_height = 0.5; //Te spremenljivke ostanejo v temu bloku, če bi hotel da so globalne bi moral definirati izven bloka
    double cylinder_radius = 0.25;

    double cylinder_pos_x = 0.9;
    double cylinder_pos_y = 0.0;
    double cylinder_pos_z = cylinder_height / 2.0;

    // Define the size of the box in meters
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.CYLINDER_HEIGHT] = cylinder_height;
    primitive.dimensions[primitive.CYLINDER_RADIUS] = cylinder_radius;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose cylinder_pose;
    cylinder_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
    cylinder_pose.position.x = cylinder_pos_x;
    cylinder_pose.position.y = cylinder_pos_y;
    cylinder_pose.position.z = cylinder_pos_z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cylinder_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  
  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object1);
  planning_scene_interface.applyCollisionObject(collision_object2);
  planning_scene_interface.applyCollisionObject(collision_object3);
  planning_scene_interface.applyCollisionObject(collision_object4);
  planning_scene_interface.applyCollisionObject(collision_object5);
  
  /*
  // Create a plan to that target pose
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  */

  for (const auto & target_pose : targets) {
    move_group_interface.setStartStateToCurrentState();
    move_group_interface.setPoseTarget(target_pose);

    auto const [success, plan] = [&move_group_interface]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
      //draw_trajectory_tool_path(plan.trajectory);
      //moveit_visual_tools.trigger();
      //prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
      //draw_title("Executing");
      //moveit_visual_tools.trigger();
      move_group_interface.execute(plan);
      move_group_interface.clearPoseTargets();
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
    } else {
      //draw_title("Planning Failed!");
      //moveit_visual_tools.trigger();
      RCLCPP_ERROR(logger, "Planning failed!");
      break;
    }
  }
  
  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}