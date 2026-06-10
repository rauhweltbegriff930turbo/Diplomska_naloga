from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fanuc_m20ia", package_name="fanuc_m20ia_moveit_config").to_moveit_configs()
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
        generate_demo_launch(moveit_config),
    ])
