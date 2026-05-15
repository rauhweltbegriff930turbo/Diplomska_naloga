from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from launch.actions import SetEnvironmentVariable #Za določitev poti do datotek
from launch.substitutions import EnvironmentVariable #Za določitev poti do datotek


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    gz_resource_path = SetEnvironmentVariable( #Za določitev poti do datotek
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
            ":",
            PathJoinSubstitution([
              FindPackageShare("fanuc_m20ia_description"),
              "..",
            ]),
        ],
    )

    gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
            ":",
            "/opt/ros/jazzy/lib",
        ],
    )


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_m20ia_moveit_config"),
                    "config",
                    "fanuc_m20ia_gazebo.urdf.xacro",
                ]
            ),
            " ",
            "initial_positions_file:=",
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_m20ia_moveit_config"),
                    "config",
                    "initial_positions.yaml",
                ]
            ),
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "fanuc_m20ia",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    fanuc_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fanuc_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    clock_bridge = Node(
      package="ros_gz_bridge",
      executable="parameter_bridge",
      arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
      output="screen",
    )


    delayed_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delayed_fanuc_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[fanuc_arm_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="empty.sdf",
                description="Gazebo world file or world name",
            ),
            gz_resource_path, #Za določitev poti do datotek
            gz_plugin_path, #Za določitev poti do datotek
            gazebo,
            clock_bridge,
            robot_state_publisher,
            spawn_robot,
            delayed_joint_state_broadcaster,
            delayed_fanuc_arm_controller,
        ]
    )