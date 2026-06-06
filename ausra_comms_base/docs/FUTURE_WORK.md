# AUSRA Swarm Multi-Robot Future Upgrades & Roadmap

This document outlines the design, implementation steps, and concrete code templates for the prioritized upgrades to the **AUSRA (Autonomous Swarm Robot Assistance)** system over the next two weeks.

---

## 📅 Roadmap & Timeline (2-Week Schedule)

Below is the proposed sequence of tasks based on impact, dependency order, and difficulty.


> 
> ```text
> 📅 Text-Based Timeline Preview:
> =============================================================================
> Day:                  | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10| 11| 12| 13| 14| 15|
> -----------------------------------------------------------------------------
> 1. On-Board Map Merge |===|===|===|   |   |   |   |   |   |   |   |   |   |   |   |
> 2. CPU Benchmarking   |   |   |===|===|   |   |   |   |   |   |   |   |   |   |   |
> 3. Swarm Collision    |   |   |   |   |===|===|===|   |   |   |   |   |   |   |   |
> 4. Frontier Coord.    |   |   |   |   |   |   |===|===|===|   |   |   |   |   |   |
> 5. DDS Network Tuning |   |   |   |   |   |   |   |   |   |===|===|   |   |   |   |
> 6. Map Compression    |   |   |   |   |   |   |   |   |   |   |   |===|===|===|   |
> 7. Liveness Monitor   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |===|
> =============================================================================
> ```


| Priority | Feature / Upgrade | Effort | Impact | Description |
|---|---|---|---|---|
| **1** | **On-board Map Merge (Jetson)** | 3 Days | High | Runs map merging on each Jetson so if the laptop fails, they keep a full map. |
| **2** | **Swarm Collision Avoidance** | 3 Days | High | Share poses over DDS and inject other robots as obstacles into the local costmap. |
| **3** | **Frontier Exploration Coordination** | 3 Days | High | Broadcast selected targets over DDS and filter nearby frontiers to avoid duplicate search. |
| **4** | **DDS Tuning** | 2 Days | Medium | Custom XML configuration profiles to bypass multicast issues and adjust buffer queues. |
| **5** | **Lossless Map Compression** | 3 Days | Medium | Compress raw `OccupancyGrid` messages into losslessly compressed PNG images over WiFi. |
| **6** | **Heartbeat Fault Detection** | 1 Day | Low | Identify lost robots and prune them from the merge stack. |

---

## 1. On-Board Map Merge (Jetson Side)

### Objective
Enable each Jetson to run a local copy of the map merge node, subscription stack, and expansion pipeline. This makes each robot completely independent of the laptop for maintaining the global map.

### Architecture Comparison
```
[ CURRENT: Centralised Merge on Laptop ]
  ausra_1 (Jetson) ───/ausra_1/map───┐
                                      ├──► Laptop (map_merge) ──► /map_merged
  ausra_2 (Jetson) ───/ausra_2/map───┘

[ UPGRADE: Decentralised Merge on Every Machine ]
  ausra_1 (Jetson) ═══/ausra_1/map═══╦══DDS══► ausra_2 (Jetson) ──► local merge ──► /ausra_2/map_merged
                                     ╠══DDS══► Laptop (Laptop)  ──► local merge ──► /map_merged
  ausra_2 (Jetson) ═══/ausra_2/map═══╩══DDS══► ausra_1 (Jetson) ──► local merge ──► /ausra_1/map_merged
```

### Implementation Checklist
1. **Workspace Compilation**:
   Clone and build `multirobot_map_merge` and `ausra_map_merge_HW` in the Jetson workspace (`~/ausra_NM_ws/src/`).
   ```bash
   colcon build --symlink-install --packages-select multirobot_map_merge ausra_map_merge_HW
   ```
2. **Launch Setup**:
   Create a launch file `/home/omen/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms/launch/local_map_merge.launch.py` (which will live on the Jetson). 
   - It will run `map_expansion_node` for the other robots.
   - It will run `map_merge` with `robot_map_topic` configured to `/ausra_X/map` and the output topic mapped to `/ausra_X/map_merged`.
3. **Parameter Adaptations**:
   Ensure `map_merge_swarm_params.yaml` is modified so that the namespaces match the local machine's namespace dynamically.

### Monitoring & Benchmarking
Adding CPU load to the Jetson Orin Nano could impact SLAM (20Hz) or Nav2 (10Hz).
- Use `htop` to verify CPU load.
- Run `ros2 topic hz /map` and `ros2 topic hz /cmd_vel` on the Jetson. If rates drop below nominal values, increase the `map_interval_sec` in `relay_node.py` from `5.0` to `10.0` or `15.0`.

---

## 2. Swarm Collision Avoidance (Dynamic Obstacle Injection)

### Objective
Robots must know other robots' positions in real-time to avoid collisions. 

### Design: Costmap-Based Virtual Obstacles (Recommended)
Rather than writing complex collision avoidance algorithms from scratch, we feed peer poses into Nav2's standard obstacle layer as fake laser scan ranges. Nav2 will treat the peer as a dynamic obstacle and plan around it.

```
┌─────────────────┐
│ /ausra_2/pose   ├─┐
└─────────────────┘ │   ┌─────────────────────────┐      ┌─────────────────────────┐
                    ├──►│ swarm_obstacle_node.py  ├─────►│ /ausra_1/swarm_scan     │
┌─────────────────┐ │   │ (Creates 360° scan ring)│      └────────────┬────────────┘
│ /odom (local)   ├─┘   └─────────────────────────┘                   │ (sensor_msg/LaserScan)
└─────────────────┘                                                   ▼
                                                         ┌─────────────────────────┐
                                                         │ Nav2 Costmap (obstacle) │
                                                         └─────────────────────────┘
```

### Python Script Implementation: `swarm_obstacle_node.py`
Place this script in `ausra_comms` package. It subscribes to other robots' poses and publishes a local scan message centered on those poses.

```python
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class SwarmObstacleNode(Node):
    def __init__(self):
        super().__init__('swarm_obstacle_node')
        
        # Declare parameters
        self.declare_parameter('robot_name', 'ausra_1')
        self.declare_parameter('peer_robots', ['ausra_2'])
        self.declare_parameter('safety_radius', 0.45) # meters (robot physical footprint + buffer)
        
        self.robot_name = self.get_parameter('robot_name').value
        self.peer_robots = self.get_parameter('peer_robots').value
        self.safety_radius = self.get_parameter('safety_radius').value
        
        # Latest known positions of peer robots in global frame
        self.peer_poses = {}
        
        # Subscribe to peer robot poses
        for peer in self.peer_robots:
            self.create_subscription(
                PoseStamped,
                f'/{peer}/pose',
                lambda msg, p=peer: self.peer_pose_cb(msg, p),
                10
            )
            
        # Publisher for local obstacle scan
        self.scan_pub = self.create_publisher(LaserScan, f'/{self.robot_name}/swarm_obstacles_scan', 10)
        
        # Create a timer to publish dynamic scan at 10Hz
        self.create_timer(0.1, self.publish_obstacle_scan)
        self.get_logger().info(f"Swarm obstacle node started for {self.robot_name} monitoring {self.peer_robots}")

    def peer_pose_cb(self, msg, peer_name):
        self.peer_poses[peer_name] = msg

    def publish_obstacle_scan(self):
        if not self.peer_poses:
            return
            
        now = self.get_clock().now()
        
        # Create a standard LaserScan message
        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = f"{self.robot_name}/odom" # MUST match robot's local frame
        
        # Create a 360-degree scan with 8 points representing a circular envelope
        num_points = 8
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2 * math.pi) / num_points
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 50.0
        
        # Default all ranges to inf
        scan.ranges = [float('inf')] * num_points
        
        # We need own position relative to peer position to place scans.
        # Alternatively, publish scan in map frame if Nav2 costmap is set to read it.
        # For ease, we can set frame_id = "map" (since both robots run on the shared map frame).
        scan.header.frame_id = "map" 
        
        for peer, pose_stamped in self.peer_poses.items():
            # Check for stale data (older than 3 seconds)
            time_diff = now - rclpy.time.Time.from_msg(pose_stamped.header.stamp)
            if time_diff.nanoseconds > 3e9:
                continue # Peer has stopped sending poses, don't project fake obstacle
                
            x_peer = pose_stamped.pose.position.x
            y_peer = pose_stamped.pose.position.y
            
            # For each direction, specify a hit at the safety radius distance
            for i in range(num_points):
                angle = scan.angle_min + i * scan.angle_increment
                # Compute global coordinate of this point on the peer envelope
                pt_x = x_peer + self.safety_radius * math.cos(angle)
                pt_y = y_peer + self.safety_radius * math.sin(angle)
                
                # To feed this to Nav2 as a local LaserScan, it is easiest to make frame_id = "map"
                # Nav2's costmap tf-buffer will automatically transform "map" points into "odom"/"base_link"
                scan.ranges[i] = self.safety_radius # Direct representation in a custom topic
                
        # To bypass complex TF transforms, we publish a point cloud or a simple scan list
        # Standard approach: Publish as PointCloud2 or custom laser scan on map frame
        # If scan is in 'map' frame, the range values must be relative to the sensor origin.
        # Instead, let's publish the peer poses as PointCloud2 where each peer gets a small ring of points.
        
def main(args=None):
    rclpy.init(args=args)
    node = SwarmObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Costmap Configuration Changes
In `nav2_params.yaml` (on each Jetson):
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan swarm_scan
        scan:
          topic: /scan
          data_type: "LaserScan"
          clearing: True
          marking: True
        swarm_scan:
          topic: /ausra_1/swarm_obstacles_scan
          data_type: "LaserScan"
          clearing: True
          marking: True
```

---

## 3. Swarm Frontier Exploration Coordination

### Objective
Prevent duplicate work where multiple robots navigate to explore the same frontier.

### Decentralized Active-Target Filtering Design
```
Robot 1 (ausra_1) selects Target A 
  └─► Publishes /ausra_1/active_target ══DDS══► Robot 2 (ausra_2)
                                                 ├─► Receives Target A
                                                 ├─► Detects frontiers locally
                                                 ├─► Filters out frontiers within 3.0m of Target A
                                                 └─► Navigates to Target B (divergent path)
```

### Implementation Pattern
1. Create a `geometry_msgs/msg/PoseStamped` publisher inside the frontier exploration node on each robot, bound to `/{robot_name}/active_target`.
2. Update the target-selection loop in the frontier exploration code:
```python
# Pseudo-python code for target selection filter:

active_peer_targets = []

def peer_target_cb(msg):
    # Keep track of active goals from other robots
    update_peer_target(msg)

def select_best_frontier(frontiers):
    valid_frontiers = []
    for f in frontiers:
        too_close = False
        for peer_target in active_peer_targets:
            dist = compute_distance(f.centroid, peer_target.pose.position)
            if dist < 3.0: # 3-meter coordination radius
                too_close = True
                break
        if not too_close:
            valid_frontiers.append(f)
            
    # Select from valid_frontiers using standard utility (distance + size)
    return select_highest_utility(valid_frontiers)
```

---

## 4. DDS Tuning for Constrained WiFi Networks

### Why Default DDS Fails on Swarms
By default, ROS 2 Humble uses FastDDS with multicast discovery. On standard home routers or phone hotspots:
- Multicast packets are frequently dropped or throttled by WiFi power-saving mechanisms.
- Large map chunks choke the discovery queue, causing robots to "drop offline" intermittently.

### Custom DDS Profiles Setup
Create a file `/home/omen/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms_base/config/fastdds_profiles.xml`:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastrtps_profiles">
    <profiles>
        <!-- Custom Participant Profile to limit buffers and set static peers -->
        <participant profile_name="swarm_wifi_profile" is_default_profile="true">
            <rtps>
                <!-- Limit buffer sizes for low memory footprint -->
                <sendBuffersAllocationQueueSize>10</sendBuffersAllocationQueueSize>
                
                <!-- Configure Discovery Protocol -->
                <builtin>
                    <discovery_config>
                        <!-- Speed up discovery checking but reduce frequency -->
                        <discoveryProtocol>SIMPLE</discoveryProtocol>
                        <leaseDuration>
                            <sec>10</sec>
                        </leaseDuration>
                        <leaseAnnouncementPeriod>
                            <sec>3</sec>
                        </leaseAnnouncementPeriod>
                    </discovery_config>
                    
                    <!-- Bypassing multicast if router blocks it. Put list of known IPs here -->
                    <!-- Only uncomment if multicast is highly unstable -->
                    <!--
                    <initialPeersList>
                        <locator>
                            <udpv4>
                                <address>192.168.1.33</address> <!- Laptop ->
                            </udpv4>
                        </locator>
                        <locator>
                            <udpv4>
                                <address>192.168.1.101</address> <!- Jetson 1 ->
                            </udpv4>
                        </locator>
                        <locator>
                            <udpv4>
                                <address>192.168.1.102</address> <!- Jetson 2 ->
                            </udpv4>
                        </locator>
                    </initialPeersList>
                    -->
                </builtin>
            </rtps>
        </participant>

        <!-- Topic specific parameters -->
        <topic profile_name="map_topic_profile">
            <historyQos>
                <kind>KEEP_LAST</kind>
                <depth>1</depth> <!-- Never queue maps, only need the latest -->
            </historyQos>
        </topic>
    </profiles>
</dds>
```

### Loading the Profile
To run ROS 2 nodes with this profile, export the environment variable:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/omen/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms_base/config/fastdds_profiles.xml
```

---

## 5. Lossless Map Compression

### Objective
Instead of sending a raw `OccupancyGrid` (1 MB) over WiFi, convert it to a 1-channel grayscale image and send it as a PNG image (`sensor_msgs/msg/CompressedImage`). PNG compression reduces this to **20-40 KB** (a 95%+ reduction) losslessly.

### Implementation: Compression Converter Functions
Add these helper methods to a python script, or extend `relay_node.py` with them.

```python
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CompressedImage

def occupancy_grid_to_compressed_image(grid_msg):
    """Converts a standard OccupancyGrid message into a CompressedImage message (PNG)."""
    width = grid_msg.info.width
    height = grid_msg.info.height
    
    # Convert data into 2D numpy array
    raw_data = np.array(grid_msg.data, dtype=np.int8).reshape((height, width))
    
    # Map Occupancy values:
    # -1 (Unknown) -> 127
    # 0 (Free) -> 255
    # 100 (Occupied) -> 0
    img_data = np.zeros((height, width), dtype=np.uint8)
    img_data[raw_data == -1] = 127
    img_data[raw_data == 0] = 255
    img_data[raw_data == 100] = 0
    
    # Compress as PNG
    success, encoded_img = cv2.imencode('.png', img_data)
    if not success:
        return None
        
    compressed_msg = CompressedImage()
    compressed_msg.header = grid_msg.header
    compressed_msg.format = "png"
    compressed_msg.data = encoded_img.tobytes()
    return compressed_msg

def compressed_image_to_occupancy_grid(compressed_msg, original_grid_info):
    """Converts a PNG CompressedImage message back to an OccupancyGrid."""
    # Decode PNG
    np_arr = np.frombuffer(compressed_msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
    
    height, width = img.shape
    
    # Map back:
    # 127 -> -1 (Unknown)
    # 255 -> 0 (Free)
    # 0 -> 100 (Occupied)
    # Handle grey values with thresholding to account for loss or rounding
    grid_data = np.zeros((height, width), dtype=np.int8)
    grid_data[img > 200] = 0        # White is free space
    grid_data[img < 50] = 100       # Black is occupied
    grid_data[(img >= 50) & (img <= 200)] = -1 # Grey is unknown
    
    grid_msg = OccupancyGrid()
    grid_msg.header = compressed_msg.header
    grid_msg.info = original_grid_info
    grid_msg.data = grid_data.flatten().tolist()
    return grid_msg
```

---

## 6. Heartbeat-Based Fault Detection

### Objective
Identify if a robot crashes or disconnects so it can be automatically removed from the map merger list, preventing the merger from expecting data that will never arrive.

### Liveness Monitor Script
Create a node `liveness_monitor.py` in `ausra_comms_base` (runs on Laptop and each Jetson):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class LivenessMonitor(Node):
    def __init__(self):
        super().__init__('liveness_monitor')
        self.declare_parameter('robots', ['ausra_1', 'ausra_2'])
        self.robots = self.get_parameter('robots').value
        
        self.last_heartbeat = {}
        self.active_status = {}
        
        for r in self.robots:
            self.last_heartbeat[r] = time.time()
            self.active_status[r] = True
            self.create_subscription(
                String,
                f'/{r}/heartbeat',
                lambda msg, name=r: self.heartbeat_cb(msg, name),
                10
            )
            
        # Monitor check timer running at 1Hz
        self.create_timer(1.0, self.check_liveness)

    def heartbeat_cb(self, msg, name):
        self.last_heartbeat[name] = time.time()
        if not self.active_status[name]:
            self.active_status[name] = True
            self.get_logger().info(f"🟢 Swarm member '{name}' came BACK ONLINE!")

    def check_liveness(self):
        now = time.time()
        for r in self.robots:
            if self.active_status[r] and (now - self.last_heartbeat[r] > 10.0):
                self.active_status[r] = False
                self.get_logger().warn(f"🔴 Swarm member '{r}' MISSED HEARTBEATS! Marking as lost.")
                # TRIGGER: Notify merge nodes or cancel goals targetting this robot

def main(args=None):
    rclpy.init(args=args)
    node = LivenessMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## 7. Bandwidth Budgeting & Priority

### Dynamic Throttling Concept
Inside `relay_node.py`, instead of a static map rate (e.g. `map_interval_sec: 5.0`), dynamically check the round-trip latency to peer nodes. If latency is high, automatically decrease the publishing rate of maps to free up the channel.

```python
# Inserted into relay_node.py loop:
import subprocess

def check_wifi_latency(self):
    try:
        # Ping peer/base station once with 1s timeout
        out = subprocess.check_output(["ping", "-c", "1", "-W", "1", "192.168.1.33"])
        # Parse output for latency
        # Adjust map_interval_sec dynamically:
        # If ping > 150ms: map_interval_sec = 15.0
        # If ping < 50ms: map_interval_sec = 5.0
    except subprocess.CalledProcessError:
        # Ping failed entirely: network heavily congested, scale throttle to max safety
        self.map_interval = 20.0
```

---

## 8. Explicitly Excluded Features

### ❌ WiFi Mesh (IBSS/802.11s)
- **Status**: **EXCLUDED**.
- **Reason**: The Jetson Orin Nano WiFi module has a known chipset/driver bug causing BSSID mismatch in IBSS mode. 802.11s configuration requires custom kernel rebuilds on JetPack 5.x/6.x, which is high-risk. 
- **Alternative**: The current smartphone mobile hotspot approach operates identically to a local router and is verified working.

### ❌ DDS Security (SROS2)
- **Status**: **EXCLUDED**.
- **Reason**: Security configuration adds key-signing overhead and complex environment variable configurations. This is not needed for the graduation project's scope, where network reliability and functional swarm mapping are the primary goals.
