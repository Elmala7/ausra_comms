# AUSRA Communication Architecture

This document explains how the AUSRA swarm robots communicate and the system design.

---

## 1. How the Robots Communicate

### The Communication Stack

```
┌──────────────────────────────────────────────────────────────┐
│                     APPLICATION LAYER                         │
│  relay_node.py — throttles map, delta detection, heartbeat   │
│  map_decompressor_node.py — reconstructs maps (when zlib on) │
├──────────────────────────────────────────────────────────────┤
│                     TRANSPORT LAYER                           │
│  Zenoh Bridge (peer mesh) — crosses WiFi, strict allowlist    │
├──────────────────────────────────────────────────────────────┤
│                     PHYSICAL LAYER                            │
│  WiFi 802.11 — all machines connected to the same router      │
└──────────────────────────────────────────────────────────────┘
```

### How Zenoh Replaces DDS Over WiFi

By default, ROS 2 uses DDS (UDP multicast) which floods the WiFi network with discovery packets for every local topic (TF, scans, etc.), causing severe network degradation.

**DDS is pinned to localhost** on every machine (`ROS_LOCALHOST_ONLY=1`). The **Zenoh bridge** is the ONLY component that communicates across WiFi. It bridges only an explicit allowlist of topics.

### What `relay_node` Does (On Each Jetson)

| Local topic | Swarm topic | Transport | Purpose |
|-------------|-------------|-----------|---------|
| `/map` or `/ausra_X/map` | `/ausra_X/map_compressed` | Zenoh | Full SLAM map, zlib-compressed (~1MB → ~1KB on sparse maps) |
| (generated) | `/ausra_X/heartbeat` | Zenoh | 1 Hz liveness check |

When compression is disabled (`enable_compression:=false`):

| Local topic | Swarm topic | Transport | Purpose |
|-------------|-------------|-----------|---------|
| `/map` or `/ausra_X/map` | `/ausra_X/map` | Zenoh | Raw OccupancyGrid (~1MB) — throttled to 1 msg / 5s |

The relay publishes the compressed topic **xor** the raw topic, never both. On the laptop, `map_decompressor_node` reconstructs `/ausra_X/map` from `/ausra_X/map_compressed`, so the map-merge pipeline is identical in both modes.

**Bandwidth Optimizations:**
1. **zlib Compression** (enabled by default): OccupancyGrids compress >99% on mostly-unknown maps. Payload is a `UInt8MultiArray` with TRANSIENT_LOCAL+RELIABLE QoS. Disable via `enable_compression:=false`.
2. **Adaptive Throttling** (enabled): Background thread pings the base station. If latency spikes (>150ms), map publish rate throttles down from 5s to 15s or 30s.
3. **Delta Detection** (enabled): Computes MD5 hash of the map data. Skips publishing if the map hasn't changed.

---

## 2. Current Architecture — Hybrid Centralised/Decentralised

### What Makes It Decentralised

| Aspect | Why it's decentralised |
|--------|----------------------|
| **SLAM** | Each robot runs its own SLAM Toolbox independently on its Jetson. If WiFi drops, the robot keeps mapping. |
| **Navigation** | Nav2 and frontier exploration run on-board. Each robot navigates autonomously. |
| **Heartbeat** | Each robot announces its own liveness. No central health monitor is required. |

### What Makes It Centralised

| Aspect | Why it's centralised |
|--------|---------------------|
| **WiFi router** | All machines connect to a single router. If the router dies, inter-robot communication stops. The router is a **single point of failure** (SPOF). |
| **Map merging** | Only the laptop merges maps into `/map_merged`. The Jetsons do NOT have a global merged map — they only know their own local SLAM map. |
| **Visualisation** | RViz2 runs only on the laptop. There is no on-board visualisation on the Jetsons. |

---

## 3. Why Each Robot Keeps Its Own Map

Even though the laptop merges all maps into `/map_merged`, **each Jetson retains its own SLAM map** (`/map` locally). This is a deliberate design choice:

1. **WiFi independence** — If the router fails, the robot continues mapping and navigating using its local map.
2. **Computational load** — Map merging is CPU-intensive. The Jetson is already busy with SLAM + Nav2 + LiDAR processing.
3. **Bandwidth** — The merged map doesn't need to flow back to the Jetsons.

### What This Means in Practice

- **Robot goes out of WiFi range**: Continues mapping and navigating autonomously. The laptop stops seeing updates.
- **WiFi comes back**: The relay_node immediately starts sending the latest map again.
- **Laptop crashes**: Both robots continue independently.

---

## 4. Summary

| Feature | Current State |
|---------|--------------|
| SLAM | ✅ On-board (decentralised) |
| Navigation | ✅ On-board (decentralised) |
| Map merge | Laptop only (centralised) |
| Transport | Zenoh bridge (peer mesh) |
| Cross-WiFi topics | `/ausra_*/map_compressed`, `/ausra_*/heartbeat` |
| Map compression | Enabled by default (UInt8MultiArray, ~99% on sparse maps) |
| Adaptive throttle | ✅ Enabled |
| Delta detection | ✅ Enabled |
