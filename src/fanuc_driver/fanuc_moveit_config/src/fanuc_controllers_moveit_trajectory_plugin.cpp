// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

// Prefer the non-obsolete .hpp headers when available, while keeping Humble compatibility.
#if __has_include("moveit_ros_control_interface/ControllerHandle.hpp")
#include "moveit_ros_control_interface/ControllerHandle.hpp"
#else
#include "moveit_ros_control_interface/ControllerHandle.h"
#endif
#if __has_include("moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.hpp")
#include "moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.hpp"
#else
#include "moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h"
#endif

#include "pluginlib/class_list_macros.hpp"

namespace moveit_ros_control_interface
{
class ScaledJointTrajectoryControllerAllocator final : public ControllerHandleAllocator
{
public:
  moveit_controller_manager::MoveItControllerHandlePtr alloc(const rclcpp::Node::SharedPtr& node,
                                                             const std::string& name,
                                                             const std::vector<std::string>& /* resources */) override
  {
    return std::make_shared<moveit_simple_controller_manager::FollowJointTrajectoryControllerHandle>(
        node, name, "follow_joint_trajectory");
  }
};

}  // namespace moveit_ros_control_interface

PLUGINLIB_EXPORT_CLASS(moveit_ros_control_interface::ScaledJointTrajectoryControllerAllocator,
                       moveit_ros_control_interface::ControllerHandleAllocator);
