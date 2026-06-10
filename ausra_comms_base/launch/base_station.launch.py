# ============================================================
# FILE: base_station.launch.py
# PACKAGE: ausra_comms_base
# RUNS ON: Laptop (base station)
# PURPOSE: Launches map_merge pipeline, RViz2, and the Zenoh
#          cross-WiFi bridge. With Zenoh enabled, robot topics
#          arrive over the Zenoh fabric (not raw DDS multicast).
#
# LAUNCH ARGUMENTS:
#   use_zenoh — start the Zenoh cross-WiFi bridge (default: true)
#               Set false to revert to plain DDS (also requires
#               unsetting ROS_LOCALHOST_ONLY=1 — see ZENOH_GUIDE.md).
#
# ENVIRONMENT:
#   ROS_LOCALHOST_ONLY=1  — enforced when use_zenoh=true (Zenoh is the
#                            only cross-machine channel).
#   ZENOH_BRIDGE_BIN      — override path to zenoh-bridge-ros2dds binary.
#                            Defaults to /opt/zenoh-bridge/zenoh-bridge-ros2dds.
# ============================================================

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

        # --- Zenoh bridge (graceful: respawn, no kill-cascade) ---
        # Starts first so the bridge is ready when subscribers come up.
        # respawn=True: bridge restarts if it crashes; the merge pipeline
        # keeps running and recovers as soon as the bridge is back.
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

        # --- Map merge: /ausra_1/map, /ausra_2/map → /map_merged ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                bringup_share,
                '/launch/map_merge.launch.py',
            ]),
        ),

        # --- RViz2: visualization ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
