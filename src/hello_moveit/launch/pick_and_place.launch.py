from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def launch_setup(context, *args, **kwargs):
    moveit_config = MoveItConfigsBuilder(
        "fanuc_m20ia",
        package_name="fanuc_m20ia_moveit_config",
    ).to_moveit_configs()

    targets_file = LaunchConfiguration("targets_file").perform(context)
    with open(targets_file, "r", encoding="utf-8") as file:
        targets_params = yaml.safe_load(file)["hello_moveit"]["ros__parameters"]

    return [
        Node(
            package="hello_moveit",
            executable="pick_and_place",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                targets_params,
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "targets_file",
                default_value=PathJoinSubstitution([
                    FindPackageShare("hello_moveit"),
                    "config",
                    "hello_moveit_targets.yaml",
                ]),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
