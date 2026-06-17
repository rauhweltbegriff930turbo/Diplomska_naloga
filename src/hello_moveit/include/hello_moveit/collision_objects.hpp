#pragma once

#include <string>
#include <vector>

#include <moveit_msgs/msg/collision_object.hpp>

namespace hello_moveit
{

std::vector<moveit_msgs::msg::CollisionObject>
makeCollisionObjects(const std::string& frame_id);

}  // namespace hello_moveit