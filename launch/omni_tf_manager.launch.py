"""Launch the profile-driven Omni TF manager."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_node(context):
    config_file = LaunchConfiguration('config_file').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    parameters = [
        config_file,
        {
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool
            ),
            'slam_state.required': ParameterValue(
                LaunchConfiguration('slam_state_required'), value_type=bool
            ),
            'slam_state.fallback_mode': LaunchConfiguration('fallback_slam_mode'),
        },
    ]
    if mode != 'profile':
        if mode not in ('shadow', 'authority'):
            raise RuntimeError('mode must be profile, shadow or authority')
        parameters.append({'mode': mode})
    return [
        Node(
            package='omni_tf_manager',
            executable='omni_tf_manager_node',
            name='omni_tf_manager',
            output='screen',
            parameters=parameters,
            arguments=['--ros-args', '--log-level', 'info'],
        )
    ]


def generate_launch_description():
    """Create the TF manager launch description."""
    package_share = Path(get_package_share_directory('omni_tf_manager'))
    default_config = package_share / 'config' / 'generic_four_sensor.yaml'

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_file',
                default_value=str(default_config),
                description='Absolute path to an omni_tf_manager robot profile',
            ),
            DeclareLaunchArgument(
                'mode',
                default_value='profile',
                description='profile uses YAML mode; shadow/authority explicitly overrides it',
            ),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument(
                'slam_state_required',
                default_value='true',
                description='Require /omni/slam/status before publishing dynamic TF',
            ),
            DeclareLaunchArgument(
                'fallback_slam_mode',
                default_value='stopped',
                description='Manual mode for standalone simulation without omni_slam_manager',
            ),
            OpaqueFunction(function=_launch_node),
        ]
    )
