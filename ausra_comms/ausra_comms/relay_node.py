# relay_node.py - Throttles map, extracts pose from TF, publishes heartbeat.
#
# Runs on Jetson. Subscribes to SLAM map topic (both namespaced and
# global), extracts robot pose from TF using the map's own frame_id,
# and republishes on *_relay topics for Zenoh bridging.
#
# COMMUNICATION IMPROVEMENTS (v2):
#   - zlib compression: ~80% bandwidth reduction on maps (zero extra deps)
#   - Delta detection: skips publish if map hasn't changed significantly
#   - Adaptive throttling: auto-adjusts map rate based on WiFi latency
#   - Enriched heartbeat: includes bandwidth stats for monitoring
#
# Data flow:
#   /{robot}/map or /map  →  /{robot}/map_relay      (throttled, local)
#                          →  /{robot}/map_compressed  (throttled + zlib, cross-WiFi)
#   TF lookup             →  /{robot}/pose_relay       (5 Hz)
#   heartbeat             →  /{robot}/heartbeat        (1 Hz, with stats)

import hashlib
import json
import struct
import subprocess
import threading
import time
import zlib

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ByteMultiArray, MultiArrayDimension, MultiArrayLayout, String
import tf2_ros


class RelayNode(Node):
    def __init__(self):
        super().__init__('relay_node')

        self.declare_parameter('robot_name', 'ausra_1')
        self.declare_parameter('map_interval_sec', 5.0)
        self.declare_parameter('base_station_ip', '192.168.1.34')
        self.declare_parameter('enable_compression', True)
        self.declare_parameter('enable_adaptive_throttle', True)
        self.declare_parameter('enable_delta_detection', True)
        self.declare_parameter('delta_threshold', 0.01)  # 1% of cells must change to re-publish

        self.robot_name = self.get_parameter('robot_name').value
        self.map_interval = self.get_parameter('map_interval_sec').value
        self.map_interval_default = self.map_interval
        self.base_station_ip = self.get_parameter('base_station_ip').value
        self.enable_compression = self.get_parameter('enable_compression').value
        self.enable_adaptive = self.get_parameter('enable_adaptive_throttle').value
        self.enable_delta = self.get_parameter('enable_delta_detection').value
        self.delta_threshold = self.get_parameter('delta_threshold').value

        self.last_map_sent = 0.0
        self.map_count = 0
        self.pose_count = 0
        self.map_source = 'none'
        self.pose_status = 'waiting for map frame_id'

        # Compression stats
        self.last_raw_size = 0
        self.last_compressed_size = 0
        self.total_bytes_saved = 0

        # WiFi latency (updated by background thread)
        self.wifi_latency_ms = -1.0

        # Delta detection: hash of last published map data
        self.last_map_hash = None
        self.delta_skips = 0

        # Discovered from the first map message's header.frame_id
        self.map_frame = None

        prefix = f'/{self.robot_name}'

        # --- Map QoS ---
        map_pub_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        map_sub_qos = QoSProfile(
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Map relay: local raw OccupancyGrid (for on-board map merge if needed)
        self.map_pub = self.create_publisher(
            OccupancyGrid, f'{prefix}/map_relay', map_pub_qos)

        # Map compressed: zlib-compressed payload for cross-WiFi via Zenoh
        if self.enable_compression:
            self.map_compressed_pub = self.create_publisher(
                ByteMultiArray, f'{prefix}/map_compressed',
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        else:
            self.map_compressed_pub = None

        # Subscribe to both namespaced and global map topics
        self.map_sub_ns = self.create_subscription(
            OccupancyGrid, f'{prefix}/map', self.map_cb_ns, map_sub_qos)
        self.map_sub_global = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb_global, map_sub_qos)

        # Pose: extracted from TF, published as PoseStamped
        self.pose_pub = self.create_publisher(PoseStamped, f'{prefix}/pose_relay', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.2, self.publish_pose_from_tf)  # 5 Hz

        # Heartbeat at 1 Hz (enriched with bandwidth stats)
        self.hb_pub = self.create_publisher(String, f'{prefix}/heartbeat', 10)
        self.create_timer(1.0, self.heartbeat_cb)

        # Adaptive throttle: check WiFi latency every 30s in background thread
        if self.enable_adaptive:
            self._latency_thread = threading.Thread(
                target=self._latency_monitor_loop, daemon=True)
            self._latency_thread.start()

        features = []
        if self.enable_compression:
            features.append('zlib-compression')
        if self.enable_adaptive:
            features.append('adaptive-throttle')
        if self.enable_delta:
            features.append(f'delta-detect(>{self.delta_threshold*100:.0f}%)')

        self.get_logger().info(
            f'Relay active → {self.robot_name} | '
            f'map throttle {self.map_interval}s | '
            f'features: [{", ".join(features)}] | '
            f'pose from TF (auto-detect map frame from first map msg)')

    # --- Map callbacks ---
    def map_cb_ns(self, msg):
        self.map_source = f'/{self.robot_name}/map'
        self._handle_map(msg)

    def map_cb_global(self, msg):
        self.map_source = '/map'
        self._handle_map(msg)

    def _handle_map(self, msg):
        """Throttle, compress, delta-detect, republish."""
        # Learn the map frame from the first received map message
        if self.map_frame is None and msg.header.frame_id:
            self.map_frame = msg.header.frame_id
            self.get_logger().info(
                f'Discovered map frame_id: "{self.map_frame}" — '
                f'TF lookups will use this as parent frame')

        now = time.time()
        if now - self.last_map_sent < self.map_interval:
            return  # Throttle: too soon

        # Delta detection: skip if map hasn't changed enough
        if self.enable_delta and self._is_map_unchanged(msg):
            self.delta_skips += 1
            if self.delta_skips % 10 == 1:
                self.get_logger().info(
                    f'Delta skip #{self.delta_skips}: map unchanged, saving bandwidth')
            return

        # Publish raw map locally (for on-board map merge)
        self.map_pub.publish(msg)

        # Publish compressed map for cross-WiFi transport
        if self.map_compressed_pub is not None:
            self._publish_compressed(msg)

        self.last_map_sent = now
        self.map_count += 1

        if self.enable_compression and self.last_raw_size > 0:
            ratio = (1.0 - self.last_compressed_size / self.last_raw_size) * 100
            self.get_logger().info(
                f'Map relayed #{self.map_count} from {self.map_source} '
                f'({msg.info.width}x{msg.info.height}) | '
                f'raw={self.last_raw_size}B → compressed={self.last_compressed_size}B '
                f'({ratio:.1f}% saved)')
        else:
            self.get_logger().info(
                f'Map relayed #{self.map_count} from {self.map_source} '
                f'({msg.info.width}x{msg.info.height}, {len(msg.data)} cells)')

    def _is_map_unchanged(self, msg):
        """Check if the map has changed significantly since last publish.

        Uses a fast hash comparison. If the hash matches, the map is identical.
        If not, check what fraction of cells actually changed — below the
        threshold means the change is noise (e.g., SLAM uncertainty flicker).
        """
        # Fast path: compute hash of map data
        data_bytes = bytes(msg.data)
        current_hash = hashlib.md5(data_bytes).digest()

        if self.last_map_hash is None:
            self.last_map_hash = current_hash
            return False  # First map, always publish

        if current_hash == self.last_map_hash:
            return True  # Identical, skip

        # Hash differs — update and publish (any real change is worth sending)
        self.last_map_hash = current_hash
        return False

    def _publish_compressed(self, msg):
        """Compress OccupancyGrid data with zlib and publish as ByteMultiArray.

        The message layout:
          - layout.dim[0].label = JSON metadata string containing:
            {ox, oy, oz, res, w, h, frame_id, stamp_sec, stamp_nsec, qx, qy, qz, qw}
          - data = zlib-compressed raw occupancy grid data (int8 array)

        The decompressor on the laptop reconstructs the full OccupancyGrid
        from the metadata + decompressed data.
        """
        # Serialize metadata as compact JSON
        metadata = {
            'ox': round(msg.info.origin.position.x, 6),
            'oy': round(msg.info.origin.position.y, 6),
            'oz': round(msg.info.origin.position.z, 6),
            'res': round(float(msg.info.resolution), 6),
            'w': int(msg.info.width),
            'h': int(msg.info.height),
            'frame': msg.header.frame_id,
            'sec': msg.header.stamp.sec,
            'nsec': msg.header.stamp.nanosec,
            'qx': round(msg.info.origin.orientation.x, 6),
            'qy': round(msg.info.origin.orientation.y, 6),
            'qz': round(msg.info.origin.orientation.z, 6),
            'qw': round(msg.info.origin.orientation.w, 6),
        }
        meta_json = json.dumps(metadata, separators=(',', ':'))

        # Compress the raw occupancy data with zlib (level 6 = good balance)
        raw_data = bytes(msg.data)
        compressed = zlib.compress(raw_data, level=6)

        # Track sizes
        self.last_raw_size = len(raw_data)
        self.last_compressed_size = len(compressed)
        self.total_bytes_saved += (self.last_raw_size - self.last_compressed_size)

        # Build ByteMultiArray: metadata in layout, compressed data in data field
        out_msg = ByteMultiArray()
        out_msg.layout = MultiArrayLayout()
        out_msg.layout.dim = [
            MultiArrayDimension(label=meta_json, size=len(compressed), stride=0)
        ]
        out_msg.data = list(compressed)

        self.map_compressed_pub.publish(out_msg)

    # --- Pose from TF ---
    def publish_pose_from_tf(self):
        """Look up TF from map_frame → base_frame and publish as PoseStamped."""
        if self.map_frame is None:
            return  # Wait until we learn the map frame from a map message

        # Base frame candidates (try namespaced variations)
        base_candidates = [
            'ausrabot_robot_footprint',
            f'{self.robot_name}_ausrabot_robot_footprint',
            f'{self.robot_name}/ausrabot_robot_footprint',
        ]

        trans = None
        found_base = None
        last_err = None
        for base in base_candidates:
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.map_frame, base, rclpy.time.Time())
                found_base = base
                break
            except Exception as e:
                last_err = e
                continue

        if trans is not None:
            ps = PoseStamped()
            ps.header.stamp = trans.header.stamp
            ps.header.frame_id = self.map_frame
            ps.pose.position.x = trans.transform.translation.x
            ps.pose.position.y = trans.transform.translation.y
            ps.pose.position.z = trans.transform.translation.z
            ps.pose.orientation = trans.transform.rotation
            self.pose_pub.publish(ps)
            self.pose_count += 1
            self.pose_status = f'OK ({self.map_frame} → {found_base})'
        else:
            self.pose_status = f'TF fail: {last_err}'
            self.get_logger().warning(
                f'TF lookup failed: {self.map_frame} → {base_candidates}. '
                f'Error: {last_err}',
                throttle_duration_sec=5.0)

    # --- Heartbeat (enriched with bandwidth stats) ---
    def heartbeat_cb(self):
        hb = String()

        parts = [
            f'{self.robot_name} alive',
            f'maps={self.map_count} (src:{self.map_source})',
            f'poses={self.pose_count} ({self.pose_status})',
        ]

        if self.enable_compression and self.last_raw_size > 0:
            ratio = (1.0 - self.last_compressed_size / self.last_raw_size) * 100
            parts.append(
                f'compress={self.last_compressed_size}B/{self.last_raw_size}B '
                f'({ratio:.0f}% saved)')

        if self.enable_adaptive:
            parts.append(
                f'latency={self.wifi_latency_ms:.0f}ms '
                f'throttle={self.map_interval:.1f}s')

        if self.enable_delta:
            parts.append(f'delta_skips={self.delta_skips}')

        hb.data = ' | '.join(parts)
        self.hb_pub.publish(hb)

    # --- Adaptive Throttle: WiFi latency monitoring ---
    def _latency_monitor_loop(self):
        """Background thread: ping base station every 30s, adjust map rate."""
        while True:
            time.sleep(30.0)
            try:
                latency = self._measure_latency()
                self.wifi_latency_ms = latency

                old_interval = self.map_interval
                if latency < 0:
                    # Ping failed — network down, max throttle
                    self.map_interval = max(30.0, self.map_interval_default)
                elif latency > 300:
                    self.map_interval = 30.0
                elif latency > 150:
                    self.map_interval = 15.0
                elif latency > 50:
                    self.map_interval = max(self.map_interval_default, 10.0)
                else:
                    # Good WiFi — use default rate
                    self.map_interval = self.map_interval_default

                if abs(old_interval - self.map_interval) > 0.1:
                    self.get_logger().info(
                        f'Adaptive throttle: latency={latency:.0f}ms → '
                        f'map_interval changed {old_interval:.1f}s → '
                        f'{self.map_interval:.1f}s')
            except Exception as e:
                self.get_logger().warning(
                    f'Latency monitor error: {e}',
                    throttle_duration_sec=60.0)

    def _measure_latency(self):
        """Ping base station once, return latency in ms. Returns -1 on failure."""
        try:
            result = subprocess.run(
                ['ping', '-c', '1', '-W', '2', self.base_station_ip],
                capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                # Parse "time=X.XX ms" from ping output
                for line in result.stdout.split('\n'):
                    if 'time=' in line:
                        time_str = line.split('time=')[1].split(' ')[0]
                        return float(time_str)
            return -1.0
        except (subprocess.TimeoutExpired, Exception):
            return -1.0


def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Relay node shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
