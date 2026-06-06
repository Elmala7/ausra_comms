# ============================================================
# FILE: map_merge.launch.py
# RUNS ON: Laptop (base station)
# PURPOSE: Launches the AUSRA map_expansion_node for each robot
#          plus multirobot_map_merge to produce /map_merged.
#
# ARCHITECTURE (adapted from ausra_map_merge_HW):
#   Each robot's /ausra_X/map (from relay_node or namespaced SLAM)
#   is fed through a map_expansion_node that stamps it onto a
#   fixed-size canvas. The merger then overlays the canvases.
#
#   /ausra_1/map → map_expansion_node → /ausra_1/map_fixed ─┐
#   /ausra_2/map → map_expansion_node → /ausra_2/map_fixed ──┼→ map_merge → /map_merged
#   /ausra_3/map → map_expansion_node → /ausra_3/map_fixed ─┘
#
# A phantom robot (ausra_99) always publishes an all-Unknown
# canvas to prevent the composeGrids segfault when fewer
# than 2 real maps are available at startup.
#
# PLACEHOLDERS TO REPLACE MANUALLY:
#   robot_offset_x / robot_offset_y — physical spawn offsets
#   for each robot, measured from a common origin point.
#   Set to 0.0 for testing, fill in real values for deployment.
# ============================================================

import os
from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ==============================================================================
# ── ROBOT CONFIG — Edit spawn offsets for your physical deployment ────────────
# robot_offset_x/y: tape-measured distance from a common origin point (metres)
# slam_topic: the topic this robot's map arrives on (from SLAM namespace or relay)
# ==============================================================================

ROBOT_CONFIG = {
    'ausra_1': {
        'offset_x': 1.0,
        'offset_y': 0.0,
        'slam_topic': '/ausra_1/map',
    },
    'ausra_2': {
        'offset_x': 0.0,
        'offset_y': 0.0,
        'slam_topic': '/ausra_2/map',
    },
    'ausra_3': {
        'offset_x': 0.0,
        'offset_y': 0.0,
        'slam_topic': '/ausra_3/map',
    },
}

# Canvas parameters (50m x 50m at 0.05m/cell = 1000x1000 grid)
CANVAS_WIDTH      = 1000
CANVAS_HEIGHT     = 1000
CANVAS_RESOLUTION = 0.05
CANVAS_ORIGIN_X   = -25.0
CANVAS_ORIGIN_Y   = -25.0

MAP_FIXED_SUFFIX  = 'map_fixed'


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('ausra_comms')
    params_file = os.path.join(pkg_share, 'config', 'map_merge_swarm_params.yaml')

    # ── Startup log ────────────────────────────────────────────────────────
    ld.add_action(LogInfo(msg=(
        '\n'
        '╔══════════════════════════════════════════════════════════════╗\n'
        '║         AUSRA Comms — Map Merge (Base Station)              ║\n'
        '╠══════════════════════════════════════════════════════════════╣\n'
        '║ INPUT:  /ausra_1/map, /ausra_2/map, /ausra_3/map            ║\n'
        '║ OUTPUT: /map_merged                                          ║\n'
        '║ CANVAS: 1000×1000 @ 0.05 m/cell | Origin (-25.0, -25.0)     ║\n'
        '╚══════════════════════════════════════════════════════════════╝\n'
    )))

    # ── Map Expansion Nodes (one per real robot) ───────────────────────────
    for robot_name, cfg in ROBOT_CONFIG.items():
        output_topic = f'/{robot_name}/{MAP_FIXED_SUFFIX}'

        ld.add_action(LogInfo(msg=(
            f'[AUSRA] {robot_name}: '
            f'SLAM topic={cfg["slam_topic"]} → {output_topic} | '
            f'offset=({cfg["offset_x"]:.3f}, {cfg["offset_y"]:.3f})'
        )))

        expansion_node = Node(
            package='ausra_map_merge_HW',
            executable='map_expansion_node',
            name=f'map_expansion_{robot_name}',
            namespace='',
            parameters=[{
                'input_topic':        cfg['slam_topic'],
                'output_topic':       output_topic,
                'robot_offset_x':     cfg['offset_x'],
                'robot_offset_y':     cfg['offset_y'],
                'canvas_width':       CANVAS_WIDTH,
                'canvas_height':      CANVAS_HEIGHT,
                'canvas_resolution':  CANVAS_RESOLUTION,
                'canvas_origin_x':    CANVAS_ORIGIN_X,
                'canvas_origin_y':    CANVAS_ORIGIN_Y,
                'publish_rate_hz':    1.0,
            }],
            output='screen',
        )
        ld.add_action(expansion_node)

    # ── Phantom Expansion Node ─────────────────────────────────────────────
    # Prevents multirobot_map_merge from segfaulting with only 1 real map.
    # Subscribes to a topic that never publishes, heartbeats all-Unknown.
    # Uses 'ausra_99' namespace which also matches the 'ausra_' namespace
    # pattern that map_merge searches for.
    phantom_node = Node(
        package='ausra_map_merge_HW',
        executable='map_expansion_node',
        name='map_expansion_phantom',
        namespace='',
        parameters=[{
            'input_topic':        '/map_phantom_never_published',
            'output_topic':       f'/ausra_99/{MAP_FIXED_SUFFIX}',
            'robot_offset_x':     0.0,
            'robot_offset_y':     0.0,
            'canvas_width':       CANVAS_WIDTH,
            'canvas_height':      CANVAS_HEIGHT,
            'canvas_resolution':  CANVAS_RESOLUTION,
            'canvas_origin_x':    CANVAS_ORIGIN_X,
            'canvas_origin_y':    CANVAS_ORIGIN_Y,
            'publish_rate_hz':    1.0,
        }],
        output='screen',
    )
    ld.add_action(phantom_node)

    # ── Central Map Merge Node ─────────────────────────────────────────────
    # Delayed 2s to let heartbeat canvases publish at least once before
    # the merger begins its discovery scan.
    map_merge_node = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg='[AUSRA] Starting multirobot_map_merge node...'),
            Node(
                package='multirobot_map_merge',
                executable='map_merge',
                name='map_merge',
                namespace='',
                parameters=[params_file],
                output='screen',
            ),
        ]
    )
    ld.add_action(map_merge_node)

    return ld
