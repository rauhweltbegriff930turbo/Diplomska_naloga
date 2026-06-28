#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
constexpr char kPlanningGroup[] = "fanuc_arm";
constexpr char kEndEffectorLink[] = "tool_tip";

class RawTerminal
{
public:
  RawTerminal()
  {
    fd_ = open("/dev/tty", O_RDWR);
    if (fd_ < 0) {
      fd_ = STDIN_FILENO;
    }

    if (tcgetattr(fd_, &original_) != 0) {
      perror("tcgetattr");
      return;
    }

    valid_ = true;
    auto raw = original_;
    raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    tcsetattr(fd_, TCSANOW, &raw);
  }

  ~RawTerminal()
  {
    if (valid_) {
      tcsetattr(fd_, TCSANOW, &original_);
    }
    if (fd_ != STDIN_FILENO) {
      close(fd_);
    }
  }

  int fd() const
  {
    return fd_;
  }

  RawTerminal(const RawTerminal &) = delete;
  RawTerminal & operator=(const RawTerminal &) = delete;

private:
  int fd_{STDIN_FILENO};
  bool valid_{false};
  termios original_{};
};

double degToRad(double degrees)
{
  return degrees * M_PI / 180.0;
}

void discardPendingInput(int fd)
{
  timeval timeout{};
  fd_set read_fds;

  while (true) {
    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);

    const int ready = select(fd + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready <= 0) {
      return;
    }

    char ignored = 0;
    if (read(fd, &ignored, 1) <= 0) {
      return;
    }
  }
}

void printHelp(double linear_step, double angular_step_deg)
{
  std::cout << "\nTool jog keyboard control\n"
            << "Planning group: " << kPlanningGroup << "\n"
            << "End effector:   " << kEndEffectorLink << "\n\n"
            << "XYZ:\n"
            << "  w/s  X +/-\n"
            << "  a/d  Y +/-\n"
            << "  r/f  Z +/-\n\n"
            << "RPY:\n"
            << "  l/j  roll +/-\n"
            << "  i/k  pitch +/-\n"
            << "  u/o  yaw +/-\n\n"
            << "Steps:\n"
            << "  +/-  linear step x2 / x0.5\n"
            << "  [/]  angular step x0.5 / x2\n\n"
            << "Other:\n"
            << "  h    help\n"
            << "  q    quit\n\n"
            << "Current steps: linear=" << linear_step << " m, angular=" << angular_step_deg
            << " deg\n"
            << "Each movement key plans and executes one MoveIt motion.\n"
            << std::endl;
}

void applyRpyDelta(geometry_msgs::msg::Pose & pose, double droll, double dpitch, double dyaw)
{
  tf2::Quaternion current;
  tf2::fromMsg(pose.orientation, current);
  current.normalize();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(current).getRPY(roll, pitch, yaw);

  tf2::Quaternion target;
  target.setRPY(roll + droll, pitch + dpitch, yaw + dyaw);
  target.normalize();
  pose.orientation = tf2::toMsg(target);
}

bool planAndExecute(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const geometry_msgs::msg::Pose & target_pose,
  const rclcpp::Logger & logger)
{
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose, kEndEffectorLink);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool planned = static_cast<bool>(move_group.plan(plan));
  if (!planned) {
    RCLCPP_WARN(logger, "Planning failed; robot was not moved.");
    move_group.clearPoseTargets();
    return false;
  }

  const auto result = move_group.execute(plan);
  move_group.clearPoseTargets();
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(logger, "Execution failed.");
    return false;
  }

  return true;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "tool_jog", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const auto logger = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface move_group(node, kPlanningGroup);
  move_group.setEndEffectorLink(kEndEffectorLink);
  move_group.setPlanningTime(5.0);
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);

  double linear_step = node->declare_parameter("linear_step", 0.01);
  double angular_step_deg = node->declare_parameter("angular_step_deg", 2.0);

  RawTerminal terminal;
  printHelp(linear_step, angular_step_deg);

  bool running = true;
  while (rclcpp::ok() && running) {
    char key = 0;
    if (read(terminal.fd(), &key, 1) <= 0) {
      continue;
    }
    discardPendingInput(terminal.fd());
    auto target_pose = move_group.getCurrentPose(kEndEffectorLink).pose;

    bool move_requested = true;
    switch (key) {
      case 'w':
        target_pose.position.x += linear_step;
        break;
      case 's':
        target_pose.position.x -= linear_step;
        break;
      case 'a':
        target_pose.position.y += linear_step;
        break;
      case 'd':
        target_pose.position.y -= linear_step;
        break;
      case 'r':
        target_pose.position.z += linear_step;
        break;
      case 'f':
        target_pose.position.z -= linear_step;
        break;
      case 'l':
        applyRpyDelta(target_pose, degToRad(angular_step_deg), 0.0, 0.0);
        break;
      case 'j':
        applyRpyDelta(target_pose, -degToRad(angular_step_deg), 0.0, 0.0);
        break;
      case 'i':
        applyRpyDelta(target_pose, 0.0, degToRad(angular_step_deg), 0.0);
        break;
      case 'k':
        applyRpyDelta(target_pose, 0.0, -degToRad(angular_step_deg), 0.0);
        break;
      case 'u':
        applyRpyDelta(target_pose, 0.0, 0.0, degToRad(angular_step_deg));
        break;
      case 'o':
        applyRpyDelta(target_pose, 0.0, 0.0, -degToRad(angular_step_deg));
        break;
      case '+':
      case '=':
        linear_step *= 2.0;
        move_requested = false;
        std::cout << "Linear step: " << linear_step << " m" << std::endl;
        break;
      case '-':
      case '_':
        linear_step *= 0.5;
        move_requested = false;
        std::cout << "Linear step: " << linear_step << " m" << std::endl;
        break;
      case '[':
        angular_step_deg *= 0.5;
        move_requested = false;
        std::cout << "Angular step: " << angular_step_deg << " deg" << std::endl;
        break;
      case ']':
        angular_step_deg *= 2.0;
        move_requested = false;
        std::cout << "Angular step: " << angular_step_deg << " deg" << std::endl;
        break;
      case 'h':
        move_requested = false;
        printHelp(linear_step, angular_step_deg);
        break;
      case 'q':
        move_requested = false;
        running = false;
        break;
      default:
        move_requested = false;
        break;
    }

    if (move_requested) {
      std::cout << "Planning..." << std::endl;
      if (planAndExecute(move_group, target_pose, logger)) {
        std::cout << "Done." << std::endl;
      }
      discardPendingInput(terminal.fd());
    }
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
