"""
decentralized_robot.launch.py
AUSRA — Fully decentralized per-robot stack (GUI-free, runs ON each Jetson).

Brings up, in order:
  1. hardware_with_comms.launch.py   — SLAM + relay_node + Zenoh bridge (existing)
  2. map_decompressor_node           — PEERS ONLY (ignore_robot:=<self>)
  3. map_expansion_node (one/robot)  — local reads /map, peers read /<peer>/map
  4. multirobot_map_merge            — namespaced → /<robot_name>/map_merged

NO RViz, NO fake_robot_pub — those stay on the laptop (ausra_comms_base).

WHY THE LOCAL EXPANSION NODE READS /map (NOT /<self>/map):
  On the Jetson, hardware SLAM publishes the GLOBAL /map topic (the relay is the
  namespacing tool, SLAM is not). In compressed mode the relay publishes ONLY
  /<self>/map_compressed — it does NOT republish raw /<self>/map. So the robot's
  own grid lives on /map (DDS loopback, free), while each PEER's grid is
  reconstructed by the decompressor onto /<peer>/map. This avoids the robot
  ever compress→decompressing its own map.

USAGE (run the SAME robot_config on every robot; only robot_name differs):
  ros2 launch ausra_comms decentralized_robot.launch.py \
      robot_name:=ausra_1 \
      robot_config:="ausra_1:0.0:0.0 ausra_2:1.5:-2.0"
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Canvas parameters — MUST match map_merge_HW_params.yaml and the laptop.
CANVAS_WIDTH      = 1000
CANVAS_HEIGHT     = 1000
CANVAS_RESOLUTION = 0.05
CANVAS_ORIGIN_X   = -25.0
CANVAS_ORIGIN_Y   = -25.0
MAP_FIXED_SUFFIX  = 'map_fixed'


def _parse_robot_config(robot_config_str, actions):
    """Parse 'name:x:y name:x:y' → {name: {offset_x, offset_y}}."""
    cfg = {}
    if robot_config_str.strip():
        for entry in robot_config_str.split():
            try:
                name, x_str, y_str = entry.split(':')
                cfg[name] = {'offset_x': float(x_str), 'offset_y': float(y_str)}
            except ValueError:
                actions.append(LogInfo(
                    msg=f'[ERROR] Invalid robot config: {entry}. Expected name:x:y'))
    return cfg


def launch_setup(context, *args, **kwargs):
    actions = []

    robot_name = LaunchConfiguration('robot_name').perform(context)
    robot_config_str = LaunchConfiguration('robot_config').perform(context)
    robot_config = _parse_robot_config(robot_config_str, actions)

    pkg_ausra_comms = get_package_share_directory('ausra_comms')
    params_file = os.path.join(
        get_package_share_directory('ausra_map_merge_HW'),
        'config', 'map_merge_HW_params.yaml')

    actions.append(LogInfo(msg=(
        '\n'
        '╔══════════════════════════════════════════════════════════════╗\n'
        '║   AUSRA DECENTRALIZED ROBOT STACK — local merge on-board    ║\n'
        f'║   robot_name = {robot_name:<20}                        ║\n'
        '╚══════════════════════════════════════════════════════════════╝\n'
    )))

    # ── 1. Existing hardware + comms bringup (SLAM + relay + Zenoh) ──────────
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ausra_comms, 'launch',
                         'hardware_with_comms.launch.py')),
        launch_arguments={
            'robot_name': robot_name,
            'use_zenoh': 'true',
            'enable_compression': 'true',
        }.items(),
    ))

    # Everything below waits for SLAM + relay + bridge to be up (~12s in the
    # included launch) before starting the merge pipeline.
    delayed = []

    # ── 2. Decompressor — PEERS ONLY (skip our own robot) ───────────────────
    all_robots = list(robot_config.keys())
    delayed.append(LogInfo(
        msg=f'[AUSRA] Decompressor active for peers of {robot_name} '
            f'(ignoring self). All robots: {all_robots}'))
    delayed.append(Node(
        package='ausra_comms',
        executable='map_decompressor_node',
        name='map_decompressor_node',
        output='screen',
        parameters=[{
            'robots': all_robots,
            'ignore_robot': robot_name,
        }],
    ))

    # ── 3. Map expansion nodes — one per robot ──────────────────────────────
    #   local robot  → input /map               (SLAM loopback)
    #   peer robots  → input /<peer>/map         (decompressor output)
    for name, cfg in robot_config.items():
        if name == robot_name:
            input_topic = '/map'
        else:
            input_topic = f'/{name}/map'
        output_topic = f'/{name}/{MAP_FIXED_SUFFIX}'

        delayed.append(LogInfo(
            msg=f'[AUSRA] expansion {name}: {input_topic} → {output_topic} '
                f'offset=({cfg["offset_x"]:.2f}, {cfg["offset_y"]:.2f})'))
        delayed.append(Node(
            package='ausra_map_merge_HW',
            executable='map_expansion_node',
            name=f'map_expansion_{name}',
            namespace='',
            output='screen',
            parameters=[{
                'input_topic':        input_topic,
                'output_topic':       output_topic,
                'robot_offset_x':     cfg['offset_x'],
                'robot_offset_y':     cfg['offset_y'],
                'canvas_width':       CANVAS_WIDTH,
                'canvas_height':      CANVAS_HEIGHT,
                'canvas_resolution':  CANVAS_RESOLUTION,
                'canvas_origin_x':    CANVAS_ORIGIN_X,
                'canvas_origin_y':    CANVAS_ORIGIN_Y,
                'publish_rate_hz':    1.0,
                'output_frame_id':    'map',
            }],
        ))

    # ── 4. Local merger — namespaced so it publishes /<robot_name>/map_merged ─
    #   init_pose_* MUST stay 0.0 (offset already baked into canvas pixels).
    dynamic_init_poses = {}
    for name in robot_config.keys():
        dynamic_init_poses[f'/{name}/map_merge/init_pose_x'] = 0.0
        dynamic_init_poses[f'/{name}/map_merge/init_pose_y'] = 0.0
        dynamic_init_poses[f'/{name}/map_merge/init_pose_z'] = 0.0
        dynamic_init_poses[f'/{name}/map_merge/init_pose_yaw'] = 0.0

    delayed.append(LogInfo(msg='[AUSRA] Starting local multirobot_map_merge...'))
    delayed.append(Node(
        package='multirobot_map_merge',
        executable='map_merge',
        name='map_merge',
        namespace=f'/{robot_name}',
        output='screen',
        parameters=[params_file, dynamic_init_poses],
    ))

    actions.append(TimerAction(period=14.0, actions=delayed))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name', default_value='ausra_1',
            description='This robot (ausra_1, ausra_2, ...)'),
        DeclareLaunchArgument(
            'robot_config',
            default_value='ausra_1:0.0:0.0 ausra_2:1.5:-2.0',
            description='Space-separated name:offset_x:offset_y for ALL robots. '
                        'Pass the SAME value on every robot.'),
        OpaqueFunction(function=launch_setup),
    ])
