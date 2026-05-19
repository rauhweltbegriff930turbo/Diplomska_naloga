from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration("robot_ip")
    group_mask = LaunchConfiguration("group_mask")
    gpio_configuration = LaunchConfiguration("gpio_configuration")
    launch_rviz = LaunchConfiguration("launch_rviz")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_m20ia_moveit_config"),
                    "config",
                    "fanuc_m20ia_physical.urdf.xacro",
                ]
            ),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "group_mask:=",
            group_mask,
            " ",
            "gpio_configuration:=",
            gpio_configuration,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    moveit_config = (
        MoveItConfigsBuilder("fanuc_m20ia", package_name="fanuc_m20ia_moveit_config")
        .robot_description(
            file_path="config/fanuc_m20ia_physical.urdf.xacro",
            mappings={
                "robot_ip": robot_ip.perform(context),
                "group_mask": group_mask.perform(context),
                "gpio_configuration": gpio_configuration.perform(context),
            },
        )
        .to_moveit_configs()
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_m20ia_moveit_config"),
                    "config",
                    "ros2_controllers_physical.yaml",
                ]
            ),
        ],
        output="both",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        arguments=[
            "--display-config",
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_m20ia_moveit_config"),
                    "config",
                    "moveit.rviz",
                ]
            ),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(launch_rviz),
    )

    controller_spawners = [
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 joint_state_broadcaster"
            ],
            shell=True,
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 fanuc_arm_controller"
            ],
            shell=True,
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 fanuc_gpio_controller"
            ],
            shell=True,
            output="screen",
        ),
    ]

    return [ros2_control_node, robot_state_publisher, move_group, rviz] + controller_spawners


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                default_value="192.168.1.100",
                description="IP address of the FANUC controller.",
            ),
            DeclareLaunchArgument(
                "gpio_configuration",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("fanuc_hardware_interface"),
                        "config",
                        "example_gpio_config_small.yaml",
                    ]
                ),
                description="GPIO configuration file used by the FANUC hardware interface.",
            ),
            DeclareLaunchArgument(
                "group_mask",
                default_value="1",
                description="RMI group mask to use when initializing remote motion.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Whether to start RViz.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
