# map_decompressor_node.py - Decompresses zlib-compressed maps from Jetsons.
#
# Runs on Laptop (base station). Subscribes to /ausra_X/map_compressed
# (ByteMultiArray with zlib payload + JSON metadata), decompresses back
# to OccupancyGrid, and publishes to /ausra_X/map_relay for the
# map_expansion_node → map_merge pipeline.
#
# Data flow:
#   Jetson relay_node → /ausra_X/map_compressed (zlib, ~30KB)
#     → Zenoh → Laptop
#       → map_decompressor_node → /ausra_X/map_relay (OccupancyGrid, ~1MB)
#         → map_expansion_node → /ausra_X/map_fixed
#           → multirobot_map_merge → /map_merged

import json
import zlib

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import ByteMultiArray
from geometry_msgs.msg import Pose, Point, Quaternion
from builtin_interfaces.msg import Time


class MapDecompressorNode(Node):
    def __init__(self):
        super().__init__('map_decompressor_node')

        self.declare_parameter('robots', ['ausra_1', 'ausra_2'])
        self.declare_parameter('ignore_robot', '')
        
        robots = self.get_parameter('robots').value
        ignore_robot = self.get_parameter('ignore_robot').value

        self.decompress_count = {r: 0 for r in robots}

        # Output QoS: transient_local + reliable so map_expansion_node
        # (with use_transient_local=false → volatile subscriber) can receive.
        # Also works if expansion node uses transient_local subscriber.
        output_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Input QoS: reliable to match relay_node's publisher
        input_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        active_robots = []
        for robot in robots:
            if robot == ignore_robot:
                self.get_logger().info(f'Ignoring local robot: {robot}')
                continue
                
            active_robots.append(robot)
            
            # Publisher: decompressed OccupancyGrid
            pub = self.create_publisher(
                OccupancyGrid, f'/{robot}/map_relay', output_qos)

            # Subscriber: compressed ByteMultiArray
            self.create_subscription(
                ByteMultiArray,
                f'/{robot}/map_compressed',
                lambda msg, r=robot, p=pub: self._decompress_cb(msg, r, p),
                input_qos)

            self.get_logger().info(
                f'Decompressor: /{robot}/map_compressed → /{robot}/map_relay')

        self.get_logger().info(
            f'Map decompressor active for {len(active_robots)} robots: {active_robots}')

    def _decompress_cb(self, msg, robot_name, publisher):
        """Decompress zlib data and reconstruct OccupancyGrid."""
        try:
            # Extract metadata from layout.dim[0].label
            if not msg.layout.dim:
                self.get_logger().error(
                    f'[{robot_name}] No metadata in compressed message layout')
                return

            meta_json = msg.layout.dim[0].label
            metadata = json.loads(meta_json)

            # Decompress the data
            compressed_data = bytes(msg.data)
            raw_data = zlib.decompress(compressed_data)

            # Verify data size matches expected width * height
            expected_size = metadata['w'] * metadata['h']
            if len(raw_data) != expected_size:
                self.get_logger().error(
                    f'[{robot_name}] Decompressed size mismatch: '
                    f'got {len(raw_data)}, expected {expected_size} '
                    f'({metadata["w"]}x{metadata["h"]})')
                return

            # Reconstruct OccupancyGrid
            grid = OccupancyGrid()

            # Header
            grid.header.frame_id = metadata['frame']
            grid.header.stamp = Time(
                sec=metadata['sec'],
                nanosec=metadata['nsec'])

            # Map metadata
            grid.info = MapMetaData()
            grid.info.resolution = float(metadata['res'])
            grid.info.width = int(metadata['w'])
            grid.info.height = int(metadata['h'])
            grid.info.origin = Pose(
                position=Point(
                    x=float(metadata['ox']),
                    y=float(metadata['oy']),
                    z=float(metadata['oz'])),
                orientation=Quaternion(
                    x=float(metadata['qx']),
                    y=float(metadata['qy']),
                    z=float(metadata['qz']),
                    w=float(metadata['qw'])))

            # Map data (convert bytes back to list of int8)
            grid.data = list(raw_data)

            publisher.publish(grid)
            self.decompress_count[robot_name] += 1

            ratio = (1.0 - len(compressed_data) / len(raw_data)) * 100
            self.get_logger().info(
                f'[{robot_name}] Decompressed map #{self.decompress_count[robot_name]}: '
                f'{len(compressed_data)}B → {len(raw_data)}B '
                f'({metadata["w"]}x{metadata["h"]}) '
                f'({ratio:.1f}% was saved)',
                throttle_duration_sec=10.0)

        except json.JSONDecodeError as e:
            self.get_logger().error(
                f'[{robot_name}] Invalid metadata JSON: {e}')
        except zlib.error as e:
            self.get_logger().error(
                f'[{robot_name}] zlib decompression failed: {e}')
        except Exception as e:
            self.get_logger().error(
                f'[{robot_name}] Decompression error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MapDecompressorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Map decompressor shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
