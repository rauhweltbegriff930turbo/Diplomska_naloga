from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "fanuc_m20ia",
        package_name="fanuc_m20ia_moveit_config",
    ).to_moveit_configs()

    return LaunchDescription(
        [
            Node(
                package="hello_moveit",
                executable="print_current_pose",
                output="screen",
                parameters=[moveit_config.to_dict()],
            ),
        ]
    )
