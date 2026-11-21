from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("web_control"),
                        "config", "web_control.yaml",
                    ]
                ),
                description="YAML parameter file for web_control.",
            ),
            Node(
                package="web_control",
                executable="web_control_node",
                name="web_control",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"use_sim_time": ParameterValue(
                        EnvironmentVariable("USE_SIM_TIME", default_value="false"),
                        value_type=bool,
                    )},
                    config_file,
                ],
            ),
        ]
    )
