import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_zenoh = LaunchConfiguration('use_zenoh')

    bringup_share = FindPackageShare('ausra_comms_base')
    pkg_base_share = get_package_share_directory('ausra_comms_base')

    zenoh_cfg = os.path.join(pkg_base_share, 'config',
                             'zenoh_bridge_laptop.json5')
    zenoh_bin = os.environ.get(
        'ZENOH_BRIDGE_BIN',
        '/opt/zenoh-bridge/zenoh-bridge-ros2dds')

    return LaunchDescription([
        DeclareLaunchArgument('use_zenoh', default_value='true',
                               description='Start Zenoh cross-WiFi bridge'),

        ExecuteProcess(
            cmd=[zenoh_bin, '-c', zenoh_cfg],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            name='zenoh_bridge',
            condition=IfCondition(use_zenoh),
        ),
        LogInfo(
            msg='>>> Zenoh bridge starting (laptop side, peer mode)...',
            condition=IfCondition(use_zenoh),
        ),

        LogInfo(msg='>>> Starting map decompressor...'),
        Node(
            package='ausra_comms_base',
            executable='map_decompressor_node',
            name='map_decompressor_node',
            output='screen',
            parameters=[{
                'robots': ['ausra_1', 'ausra_2'],
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                bringup_share,
                '/launch/map_merge.launch.py',
            ]),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
