#include <memory>
#include <thread>

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

    auto const pose = move_group_interface.getCurrentPose("tool_tip");

    RCLCPP_INFO(node->get_logger(), "Frame: %s", pose.header.frame_id.c_str());

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
