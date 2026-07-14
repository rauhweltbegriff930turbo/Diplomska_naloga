#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto const node = std::make_shared<rclcpp::Node>(
        "print_current_pose",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group_interface(node, "fanuc_arm");
    move_group_interface.setEndEffectorLink("tool_tip");

    RCLCPP_INFO(
        node->get_logger(),
        "Planning frame: %s",
        move_group_interface.getPlanningFrame().c_str());

    RCLCPP_INFO(
        node->get_logger(),
        "Pose reference frame: %s",
        move_group_interface.getPoseReferenceFrame().c_str());

    RCLCPP_INFO(
        node->get_logger(),
        "End effector link: %s",
        move_group_interface.getEndEffectorLink().c_str());

    auto const joint_names = move_group_interface.getJointNames();
    auto const joint_values = move_group_interface.getCurrentJointValues();

    for (size_t i = 0; i < joint_names.size() && i < joint_values.size(); ++i) {
        RCLCPP_INFO(
            node->get_logger(),
            "Joint %s: %.6f",
            joint_names[i].c_str(),
            joint_values[i]);
    }

    auto const pose = move_group_interface.getCurrentPose("tool_tip");

    RCLCPP_INFO(node->get_logger(), "Pose frame: %s", pose.header.frame_id.c_str());

    RCLCPP_INFO(
        node->get_logger(),
        "Position: x=%.6f y=%.6f z=%.6f",
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z);

    RCLCPP_INFO(
        node->get_logger(),
        "Orientation quaternion: x=%.6f y=%.6f z=%.6f w=%.6f",
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w);

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
