import os
from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

CANVAS_WIDTH      = 1000
CANVAS_HEIGHT     = 1000
CANVAS_RESOLUTION = 0.05
CANVAS_ORIGIN_X   = -25.0
CANVAS_ORIGIN_Y   = -25.0
MAP_FIXED_SUFFIX  = 'map_fixed'


def launch_setup(context, *args, **kwargs):
    actions = []
    pkg_share = get_package_share_directory('ausra_comms_base')
    params_file = os.path.join(pkg_share, 'config', 'map_merge_swarm_params.yaml')

    robot_config_str = LaunchConfiguration('robot_config').perform(context)
    robot_config = {}

    if robot_config_str.strip():
        for entry in robot_config_str.split():
            try:
                name, x_str, y_str = entry.split(':')
                robot_config[name] = {
                    'offset_x': float(x_str),
                    'offset_y': float(y_str),
                }
            except ValueError:
                actions.append(LogInfo(msg=f'[ERROR] Invalid robot config: {entry}. Expected name:x:y'))

    robot_count = len(robot_config)

    actions.append(LogInfo(msg=(
        '\n'
        '╔══════════════════════════════════════════════════════════════╗\n'
        '║         AUSRA Base Station — Map Merge Pipeline             ║\n'
        '╚══════════════════════════════════════════════════════════════╝\n'
    )))

    for robot_name, cfg in robot_config.items():
        slam_topic = f'/{robot_name}/map_relay'
        output_topic = f'/{robot_name}/{MAP_FIXED_SUFFIX}'

        actions.append(LogInfo(msg=(
            f'[AUSRA] {robot_name}: '
            f'{slam_topic} → {output_topic}'
        )))

        expansion_node = Node(
            package='ausra_map_merge_HW',
            executable='map_expansion_node',
            name=f'map_expansion_{robot_name}',
            namespace='',
            parameters=[{
                'input_topic':        slam_topic,
                'output_topic':       output_topic,
                'robot_offset_x':     cfg['offset_x'],
                'robot_offset_y':     cfg['offset_y'],
                'canvas_width':       CANVAS_WIDTH,
                'canvas_height':      CANVAS_HEIGHT,
                'canvas_resolution':  CANVAS_RESOLUTION,
                'canvas_origin_x':    CANVAS_ORIGIN_X,
                'canvas_origin_y':    CANVAS_ORIGIN_Y,
                'publish_rate_hz':    1.0,
                'use_transient_local': False,
            }],
            output='screen',
        )
        actions.append(expansion_node)

    dynamic_init_poses = {}
    for robot_name in robot_config.keys():
        dynamic_init_poses[f'/{robot_name}/map_merge/init_pose_x'] = 0.0
        dynamic_init_poses[f'/{robot_name}/map_merge/init_pose_y'] = 0.0
        dynamic_init_poses[f'/{robot_name}/map_merge/init_pose_z'] = 0.0
        dynamic_init_poses[f'/{robot_name}/map_merge/init_pose_yaw'] = 0.0

    map_merge_node = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg='[AUSRA] Starting multirobot_map_merge node...'),
            Node(
                package='multirobot_map_merge',
                executable='map_merge',
                name='map_merge',
                namespace='',
                parameters=[params_file, dynamic_init_poses],
                output='screen',
            ),
        ]
    )
    actions.append(map_merge_node)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config',
            default_value='ausra_1:0.0:0.0 ausra_2:0.0:0.0',
            description='Space-separated list: name:offset_x:offset_y'
        ),
        OpaqueFunction(function=launch_setup),
    ])
