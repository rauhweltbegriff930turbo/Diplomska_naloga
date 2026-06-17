#include "hello_moveit/collision_objects.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace hello_moveit
{

moveit_msgs::msg::CollisionObject makeBox(
    const std::string& frame_id,
    const std::string& id,
    double box_x,
    double box_y,
    double box_z,
    double box_pos_x,
    double box_pos_y,
    double box_pos_z)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = box_x;
  primitive.dimensions[primitive.BOX_Y] = box_y;
  primitive.dimensions[primitive.BOX_Z] = box_z;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = box_pos_x;
  box_pose.position.y = box_pos_y;
  box_pose.position.z = box_pos_z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}

moveit_msgs::msg::CollisionObject makeCylinder(
    const std::string& frame_id,
    const std::string& id,
    double cylinder_height,
    double cylinder_radius,
    double cylinder_pos_x,
    double cylinder_pos_y,
    double cylinder_pos_z)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[primitive.CYLINDER_HEIGHT] = cylinder_height;
  primitive.dimensions[primitive.CYLINDER_RADIUS] = cylinder_radius;

  geometry_msgs::msg::Pose cylinder_pose;
  cylinder_pose.orientation.w = 1.0;
  cylinder_pose.position.x = cylinder_pos_x;
  cylinder_pose.position.y = cylinder_pos_y;
  cylinder_pose.position.z = cylinder_pos_z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(cylinder_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}

std::vector<moveit_msgs::msg::CollisionObject>
makeCollisionObjects(const std::string& frame_id)
{
  std::vector<moveit_msgs::msg::CollisionObject> objects;

  {
    double box_x = 3.0;
    double box_y = 0.3;
    double box_z = 2.5;

    double box_pos_x = 0.6;
    double box_pos_y = 0.9 + (box_y / 2.0);
    double box_pos_z = box_z / 2.0;

    objects.push_back(makeBox(
        frame_id,
        "box1",
        box_x, box_y, box_z,
        box_pos_x, box_pos_y, box_pos_z));
  }

  {
    double box_x = 0.3;
    double box_y = 0.5;
    double box_z = 2.5;

    double box_pos_x = -0.3 - (box_x / 2.0);
    double box_pos_y = 0.4 + (box_y / 2.0);
    double box_pos_z = box_z / 2.0;

    objects.push_back(makeBox(
        frame_id,
        "box2",
        box_x, box_y, box_z,
        box_pos_x, box_pos_y, box_pos_z));
  }

  {
    double box_x = 0.3;
    double box_y = 2.2;
    double box_z = 2.5;

    double box_pos_x = -0.6 - (box_x / 2.0);
    double box_pos_y = -0.2;
    double box_pos_z = box_z / 2.0;

    objects.push_back(makeBox(
        frame_id,
        "box3",
        box_x, box_y, box_z,
        box_pos_x, box_pos_y, box_pos_z));
  }

  {
    double box_x = 2.5;
    double box_y = 2.5;
    double box_z = 0.15;

    double box_pos_x = 0.35;
    double box_pos_y = -0.05;
    double box_pos_z = 2.35 + (box_z / 2.0);

    objects.push_back(makeBox(
        frame_id,
        "box4",
        box_x, box_y, box_z,
        box_pos_x, box_pos_y, box_pos_z));
  }

  {
    double box_x = 2.5;
    double box_y = 2.5;
    double box_z = 0.15;

    double box_pos_x = 0.35;
    double box_pos_y = -0.05;
    double box_pos_z = -box_z / 2.0;

    objects.push_back(makeBox(
        frame_id,
        "box5",
        box_x, box_y, box_z,
        box_pos_x, box_pos_y, box_pos_z));
  }

  {
    double cylinder_height = 0.5;
    double cylinder_radius = 0.25;

    double cylinder_pos_x = 0.9;
    double cylinder_pos_y = 0.0;
    double cylinder_pos_z = cylinder_height / 2.0;

    objects.push_back(makeCylinder(
        frame_id,
        "cylinder1",
        cylinder_height,
        cylinder_radius,
        cylinder_pos_x,
        cylinder_pos_y,
        cylinder_pos_z));
  }

  return objects;
}

}  // namespace hello_moveit
