#pragma once

#include <string>
#include <vector>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>


namespace hello_moveit
{

std::vector<moveit_msgs::msg::CollisionObject>
makeCollisionObjects(const std::string& frame_id);

std::vector<moveit_msgs::msg::ObjectColor>
makeCollisionObjectColors();

}  // namespace hello_moveit