# AUSRA System Architecture & Data Flow

This document explains how the components of the AUSRA swarm system work together in the 2-Jetson + Laptop deployment.

---

## 1. ROS 2 DDS — The Communication Layer

In ROS 2, the network layer (Data Distribution Service / DDS) handles cross-machine topic sharing natively. If two machines meet these conditions:
1. Connected to the **same WiFi network** (same subnet)
2. Using the **same `ROS_DOMAIN_ID`** (we use `0`)
3. Multicast is allowed on the router

...then **they automatically see each other's topics**. No bridge or central broker is needed.

---

## 2. Production Mode — 2 Jetsons + Laptop

### Jetson Side (both Jetsons run the same software)

**Launch command:**
```bash
ros2 launch ausra_comms hardware_with_comms.launch.py robot_name:=ausra_X
```

Each Jetson runs:
1. **Hardware stack** (`hardware_full_stack.launch.py` from `lidar_slam_pkg`) — LiDAR, EKF, SLAM, Nav2, exploration
2. **Relay node** (`relay_node.py` from `ausra_comms`) — namespaces and throttles topics for DDS

The relay_node:
- Subscribes to `/map` → publishes `/ausra_X/map` (throttled to 1 msg / 5 sec)
- Subscribes to `/pose` → publishes `/ausra_X/pose` (pass-through)
- Publishes `/ausra_X/heartbeat` at 1 Hz

### Laptop Side (base station)

**Launch command:**
```bash
./start_base.sh
```

The laptop runs:
1. **Map expansion nodes** (from `ausra_map_merge_HW`) — one per robot, stamps each robot's SLAM map onto a fixed-size canvas
2. **Phantom node** — publishes an all-Unknown canvas to prevent merge crashes
3. **multirobot_map_merge** (from `m-explore-ros2`) — overlays all canvases into `/map_merged`
4. **RViz2** — visualization

### Full Data Flow

```
[ JETSON 1 — ausra_1 ]                           [ WiFi / DDS ]      [ LAPTOP — Base Station ]

SLAM ──► /map  ──► relay_node ──(throttle)──► /ausra_1/map  ════►  map_expansion_node ──► /ausra_1/map_fixed ──┐
     ──► /pose ──► relay_node ──────────────► /ausra_1/pose ════►                                               │
                    relay_node (1Hz) ────────► /ausra_1/hb   ════►                                               ├──► map_merge ──► /map_merged
                                                                                                                │
[ JETSON 2 — ausra_2 ]                                                                                         │
                                                                                                                │
SLAM ──► /map  ──► relay_node ──(throttle)──► /ausra_2/map  ════►  map_expansion_node ──► /ausra_2/map_fixed ──┤
     ──► /pose ──► relay_node ──────────────► /ausra_2/pose ════►                                               │
                    relay_node (1Hz) ────────► /ausra_2/hb   ════►                                               │
                                                                                                                │
                                                                   phantom_node ──────► /ausra_99/map_fixed ────┘
```

---

## 3. Why Relay Node Instead of ROS 2 Namespaces

ROS 2 can automatically prefix all topics via `--namespace`:
```bash
ros2 launch ... --ros-args --namespace ausra_1
```

This would turn `/cmd_vel` → `/ausra_1/cmd_vel`, `/scan` → `/ausra_1/scan`, etc.

**Why we don't do this:** If the entire hardware stack is namespaced, DDS broadcasts ALL topics over WiFi — including `/ausra_1/scan` (LiDAR point cloud, ~500 KB/s) and `/ausra_1/odom` (100 Hz). This would saturate the WiFi link.

The relay_node acts as a **firewall**: only 3 lightweight topics (`/map`, `/pose`, `/heartbeat`) cross the WiFi boundary. All internal heavy topics stay local on the Jetson.

---

## 4. Package Overview

### `ausra_comms` — Jetson Package

| File | Purpose |
|------|---------|
| `ausra_comms/relay_node.py` | Throttles & namespaces SLAM topics |
| `launch/hardware_with_comms.launch.py` | Top-level Jetson launch (hardware + relay) |

### `ausra_comms_base` — Laptop Package

| File | Purpose |
|------|---------|
| `ausra_comms_base/fake_robot_pub.py` | Dummy robot publisher (fallback testing) |
| `launch/map_merge.launch.py` | Map expansion + merge pipeline |
| `launch/base_station.launch.py` | Map merge + RViz2 |
| `config/map_merge_swarm_params.yaml` | Merge engine config |
| `scripts/start_base.sh` | Convenience startup script |

### Supporting Packages (Laptop only)

| Package | Provides |
|---------|----------|
| `ausra_map_merge_HW` | `map_expansion_node` — stamps maps onto fixed canvas |
| `m-explore-ros2` / `multirobot_map_merge` | Merge engine — overlays canvases |
