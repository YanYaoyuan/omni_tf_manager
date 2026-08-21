"""Launch the profile-driven Omni TF manager."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the TF manager launch description."""
    package_share = Path(get_package_share_directory("omni_tf_manager"))
    default_config = package_share / "config" / "generic_four_sensor.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(default_config),
                description="Absolute path to an omni_tf_manager robot profile",
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="shadow",
                description="shadow validates without TF output; authority owns configured edges",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="omni_tf_manager",
                executable="omni_tf_manager_node",
                name="omni_tf_manager",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "mode": LaunchConfiguration("mode"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
                arguments=["--ros-args", "--log-level", "info"],
            ),
        ]
    )
