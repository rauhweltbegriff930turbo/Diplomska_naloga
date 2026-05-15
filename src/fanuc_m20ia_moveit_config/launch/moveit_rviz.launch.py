from launch import LaunchDescription
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fanuc_m20ia", package_name="fanuc_m20ia_moveit_config").to_moveit_configs()
    return LaunchDescription([
          SetParameter(name="use_sim_time", value=True),
          generate_moveit_rviz_launch(moveit_config),
      ])