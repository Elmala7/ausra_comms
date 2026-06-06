# ============================================================
# FILE: base_station.launch.py
# PACKAGE: ausra_comms_base
# RUNS ON: Laptop (base station)
# PURPOSE: Launches map_merge pipeline and RViz2.
#          Robot topics arrive via ROS2 DDS automatically.
# ============================================================

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_share = FindPackageShare('ausra_comms_base')

    return LaunchDescription([
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
