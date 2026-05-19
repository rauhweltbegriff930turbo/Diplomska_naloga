# SPDX-FileCopyrightText: 2025, FANUC America Corporation
# SPDX-FileCopyrightText: 2025, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    gpio_configuration = LaunchConfiguration("gpio_configuration")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_mock = LaunchConfiguration("use_mock")

    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_m20ia_moveit_config"),
                    "launch",
                    "real_robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "gpio_configuration": gpio_configuration,
            "launch_rviz": launch_rviz,
        }.items(),
        condition=UnlessCondition(use_mock),
    )

    mock_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_m20ia_moveit_config"),
                    "launch",
                    "demo.launch.py",
                ]
            )
        ),
        condition=IfCondition(use_mock),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_model",
                default_value="m20ia",
                description="Kept for CLI compatibility. Only m20ia is supported by this wrapper.",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="192.168.2.10",
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
                "launch_rviz",
                default_value="true",
                description="Whether to start RViz.",
            ),
            DeclareLaunchArgument(
                "use_mock",
                default_value="false",
                description="Use the M-20iA MoveIt demo launch instead of the real robot launch.",
            ),
            real_robot_launch,
            mock_launch,
        ]
    )
