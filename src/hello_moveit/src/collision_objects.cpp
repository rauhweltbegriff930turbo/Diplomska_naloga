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

  moveit_msgs::msg::ObjectColor makeColor(
      const std::string& id,
      float r,
      float g,
      float b,
      float a)
  {
    moveit_msgs::msg::ObjectColor color;
    color.id = id;
    color.color.r = r;
    color.color.g = g;
    color.color.b = b;
    color.color.a = a;
    return color;
  }

std::vector<moveit_msgs::msg::CollisionObject>
makeCollisionObjects(const std::string& frame_id)
{
  std::vector<moveit_msgs::msg::CollisionObject> objects;

  {
    double box_x = 0.3;   //ovira
    double box_y = 0.1;
    double box_z = 0.45;

    double box_pos_x = 0.97;
    double box_pos_y = 0.275;
    double box_pos_z = 0.85;

    objects.push_back(makeBox(
        frame_id,
        "box0",
        box_x, box_y, box_z,
        box_pos_x, box_pos_y, box_pos_z));
  }

  {
    double box_x = 2.5;   //desna stena
    double box_y = 0.3;
    double box_z = 2.25;

    double box_pos_x = 0.5;
    double box_pos_y = 0.9 + (box_y / 2.0);
    double box_pos_z = box_z / 2.0;

    objects.push_back(makeBox(
        frame_id,
        "box1",
        box_x, box_y, box_z,
        box_pos_x, box_pos_y, box_pos_z));
  }

  {
    double box_x = 0.15;   //steber
    double box_y = 0.5;
    double box_z = 2.25;

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
    double box_x = 0.3;   //stena zadaj
    double box_y = 2.2;
    double box_z = 2.25;

    double box_pos_x = -0.45 - (box_x / 2.0);
    double box_pos_y = -0.2;
    double box_pos_z = box_z / 2.0;

    objects.push_back(makeBox(
        frame_id,
        "box3",
        box_x, box_y, box_z,
        box_pos_x, box_pos_y, box_pos_z));
  }

  {
    double box_x = 2.5;   //strop
    double box_y = 2.5;
    double box_z = 0.15;

    double box_pos_x = 0.5;
    double box_pos_y = -0.05;
    double box_pos_z = 2.25 + (box_z / 2.0);

    objects.push_back(makeBox(
        frame_id,
        "box4",
        box_x, box_y, box_z,
        box_pos_x, box_pos_y, box_pos_z));
  }

  {
    double box_x = 2.5;   //tla
    double box_y = 2.5;
    double box_z = 0.15;

    double box_pos_x = 0.5;
    double box_pos_y = -0.05;
    double box_pos_z = -box_z / 2.0;

    objects.push_back(makeBox(
        frame_id,
        "box5",
        box_x, box_y, box_z,
        box_pos_x, box_pos_y, box_pos_z));
  }

  {
    double box_x = 0.7;   //robotov krmilnik
    double box_y = 0.6;
    double box_z = 2.25;

    double box_pos_x = -0.45 + box_x/2.0;
    double box_pos_y = -0.7 - box_y/2.0;
    double box_pos_z = box_z / 2.0;

    objects.push_back(makeBox(
        frame_id,
        "box6",
        box_x, box_y, box_z,
        box_pos_x, box_pos_y, box_pos_z));
  }

  {
    double box_x = 0.4;   //robotov krmilnik
    double box_y = 0.9;
    double box_z = 0.6;

    double box_pos_x = 0.89;
    double box_pos_y = 0.0;
    double box_pos_z = box_z / 2.0;

    objects.push_back(makeBox(
        frame_id,
        "box7",
        box_x, box_y, box_z,
        box_pos_x, box_pos_y, box_pos_z));
  }

  {
    double box_x = 0.8;   //robotov krmilnik
    double box_y = 0.8;
    double box_z = 0.0948;

    double box_pos_x = 0.0;
    double box_pos_y = 0.0;
    double box_pos_z = box_z / 2.0;

    objects.push_back(makeBox(
        frame_id,
        "box8",
        box_x, box_y, box_z,
        box_pos_x, box_pos_y, box_pos_z));
  }

  {
    double cylinder_height = 0.665;   //miza
    double cylinder_radius = 0.275;

    double cylinder_pos_x = 0.89;
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

std::vector<moveit_msgs::msg::ObjectColor>
makeCollisionObjectColors()
{
  std::vector<moveit_msgs::msg::ObjectColor> colors;

  colors.push_back(makeColor("box0", 1.0f, 0.0f, 0.0f, 1.0f));      // rdeča
  colors.push_back(makeColor("box1", 0.0f, 1.0f, 0.0f, 0.6f));      // modra
  colors.push_back(makeColor("box2", 0.0f, 1.0f, 0.0f, 0.6f));      // zelena
  colors.push_back(makeColor("box3", 0.0f, 1.0f, 0.0f, 0.6f));      // zelena
  colors.push_back(makeColor("box4", 0.0f, 1.0f, 0.0f, 0.6f));      // zelena
  colors.push_back(makeColor("box5", 0.0f, 1.0f, 0.0f, 0.6f));      // zelena
  colors.push_back(makeColor("box6", 0.0f, 1.0f, 0.0f, 0.6f));      // zelena
  colors.push_back(makeColor("box7", 1.0f, 1.0f, 0.0f, 1.0f));      // zelena
  colors.push_back(makeColor("box8", 0.0f, 0.0f, 0.0f, 1.0f));      // zelena
  colors.push_back(makeColor("cylinder1", 1.0f, 1.0f, 0.0f, 1.0f)); // rumena

  return colors;
}

}  // namespace hello_moveit
