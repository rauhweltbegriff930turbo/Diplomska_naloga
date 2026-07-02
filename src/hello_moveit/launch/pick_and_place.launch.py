from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "fanuc_m20ia",
        package_name="fanuc_m20ia_moveit_config",
    ).to_moveit_configs()
    
    targets_file = LaunchConfiguration("targets_file")

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
            Node(
                package="hello_moveit",
                executable="pick_and_place",
                output="screen",
                parameters=[moveit_config.to_dict(),
                            targets_file,
                            ],
            ),
        ]
    )
