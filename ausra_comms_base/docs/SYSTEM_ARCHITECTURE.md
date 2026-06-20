# AUSRA System Architecture & Data Flow

This document explains how the components of the AUSRA swarm system work together in the 2-Jetson + Laptop deployment.

---

## 1. Communication Layer — Zenoh over WiFi

DDS is pinned to localhost on every machine (`ROS_LOCALHOST_ONLY=1`). Cross-machine communication uses `zenoh-bridge-ros2dds`, which bridges only an explicit allowlist of topics.

---

## 2. Production Mode — 2 Jetsons + Laptop

### Jetson Side (both Jetsons run the same software)

**Launch command:**
```bash
ros2 launch ausra_comms hardware_with_comms.launch.py robot_name:=ausra_X
```

Each Jetson runs:
1. **Hardware stack** (`hardware_full_stack.launch.py` from `lidar_slam_pkg`) — LiDAR, EKF, SLAM, Nav2, exploration
2. **Relay node** (`relay_node.py` from `ausra_comms`) — namespaces and throttles map for Zenoh transport
3. **Zenoh bridge** — bridges allowlisted topics to other machines

The relay_node:
- Subscribes to `/map` and `/ausra_X/map` → publishes `/ausra_X/map` (throttled to 1 msg / 5 sec)
- Publishes `/ausra_X/heartbeat` at 1 Hz

### Laptop Side (base station)

**Launch command:**
```bash
./start_base.sh
```

The laptop runs:
1. **Zenoh bridge** — receives topics from Jetsons
2. **Map decompressor** (for when compression is enabled) — decompresses zlib maps back to OccupancyGrid
3. **Map expansion nodes** (from `ausra_map_merge_HW`) — one per robot, stamps each robot's SLAM map onto a fixed-size canvas
4. **multirobot_map_merge** (from `m-explore-ros2`) — overlays all canvases into `/map_merged`
5. **RViz2** — visualization

### Full Data Flow

```
[ JETSON 1 — ausra_1 ]                              [ Zenoh ]           [ LAPTOP — Base Station ]

SLAM ──► /map ──► relay_node ──(throttle)──► /ausra_1/map  ═══►  map_expansion_node ──► /ausra_1/map_fixed ──┐
                   relay_node (1Hz) ─────────► /ausra_1/heartbeat ═══►                                              │
                                                                                                                    ├──► map_merge ──► /map_merged
[ JETSON 2 — ausra_2 ]                                                                                             │
                                                                                                                    │
SLAM ──► /map ──► relay_node ──(throttle)──► /ausra_2/map  ═══►  map_expansion_node ──► /ausra_2/map_fixed ──┘
                   relay_node (1Hz) ─────────► /ausra_2/heartbeat ═══►
```

---

## 3. Why Relay Node Instead of ROS 2 Namespaces

ROS 2 can automatically prefix all topics via `--namespace`. This would turn `/cmd_vel` → `/ausra_1/cmd_vel`, `/scan` → `/ausra_1/scan`, etc.

**Why we don't do this:** If the entire hardware stack is namespaced, DDS broadcasts ALL topics over WiFi — including `/ausra_1/scan` (LiDAR point cloud, ~500 KB/s) and `/ausra_1/odom` (100 Hz). This would saturate the WiFi link.

The relay_node acts as a **firewall**: only 2 lightweight topics (`/map`, `/heartbeat`) cross the WiFi boundary via Zenoh. All internal heavy topics stay local on the Jetson.

---

## 4. Package Overview

### `ausra_comms` — Jetson Package

| File | Purpose |
|------|---------|
| `ausra_comms/relay_node.py` | Throttles & namespaces SLAM map, publishes heartbeat |
| `ausra_comms/map_decompressor_node.py` | Decompresses peer robot maps (when compression enabled) |
| `launch/hardware_with_comms.launch.py` | Top-level Jetson launch (hardware + relay + Zenoh bridge) |
| `config/zenoh_bridge_jetson.json5` | Zenoh allowlist for Jetson |

### `ausra_comms_base` — Laptop Package

| File | Purpose |
|------|---------|
| `ausra_comms_base/fake_robot_pub.py` | Dummy robot publisher (fallback testing) |
| `ausra_comms_base/map_decompressor_node.py` | Decompresses maps from Jetsons (when compression enabled) |
| `launch/base_station.launch.py` | Zenoh bridge + decompressor + map merge + RViz2 |
| `launch/map_merge.launch.py` | Map expansion nodes + merge engine |
| `config/map_merge_swarm_params.yaml` | Merge engine config |
| `config/zenoh_bridge_laptop.json5` | Zenoh allowlist for laptop |
| `scripts/start_base.sh` | Convenience startup script |

### Supporting Packages (Laptop only)

| Package | Provides |
|---------|----------|
| `ausra_map_merge_HW` | `map_expansion_node` — stamps maps onto fixed canvas |
| `m-explore-ros2` / `multirobot_map_merge` | Merge engine — overlays canvases |
