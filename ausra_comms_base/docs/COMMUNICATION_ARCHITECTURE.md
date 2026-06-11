# AUSRA Communication Architecture

This document explains how the AUSRA swarm robots communicate, why the current design is a **hybrid centralised/decentralised** approach, and how the system can evolve toward full decentralisation in the future.

---

## 1. How the Robots Communicate

### The Communication Stack

### The Communication Stack (v2)

```
┌──────────────────────────────────────────────────────────────┐
│                     APPLICATION LAYER                         │
│  relay_node.py — throttles, compress (zlib), delta detection  │
│  map_decompressor_node.py — reconstructs maps on receiver     │
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

In the AUSRA v2 architecture, **DDS is pinned to localhost** on every machine (`ROS_LOCALHOST_ONLY=1`). The **Zenoh bridge** is the ONLY component that communicates across WiFi. It acts as a strict firewall, only bridging an explicit allowlist of topics between the Jetsons and the base station.

### What `relay_node` Does (On Each Jetson)

The relay_node is a highly optimized bandwidth management layer between the on-board SLAM and the WiFi network:

| Local topic | Swarm topic | Transport | Purpose |
|-------------|-------------|-----------|---------|
| `/map` | `/ausra_X/map_compressed` | Zenoh (zlib) | Full SLAM map — compressed from ~1MB down to ~30KB |
| `/map` | `/ausra_X/map_relay` | Local only | Raw map fallback for on-board map merge |
| TF lookup | `/ausra_X/pose_relay` | Zenoh | Robot position (~5 Hz) |
| (generated) | `/ausra_X/heartbeat` | Zenoh | 1 Hz liveness check + bandwidth stats |

**Bandwidth Optimizations:**
1. **zlib Compression:** OccupancyGrids compress extremely well (mostly -1 "Unknown" cells), reducing map size by ~80-95%.
2. **Adaptive Throttling:** A background thread pings the base station. If latency spikes (>150ms), the map publish rate automatically throttles down from 5s to 15s or 30s.
3. **Delta Detection:** Computes an MD5 hash of the map. If the map hasn't changed since the last tick, it skips publishing entirely.

---

## 2. Current Architecture — Hybrid Centralised/Decentralised

### What Makes It Decentralised

| Aspect | Why it's decentralised |
|--------|----------------------|
| **SLAM** | Each robot runs its own SLAM Toolbox independently on its Jetson. If WiFi drops, the robot keeps mapping. |
| **Navigation** | Nav2 and frontier exploration run on-board. Each robot navigates autonomously. |
| **DDS transport** | DDS is peer-to-peer at the data level — no central message broker. Topics flow directly from publisher to subscriber. |
| **Heartbeat** | Each robot announces its own liveness. No central health monitor is required. |

### What Makes It Centralised

| Aspect | Why it's centralised |
|--------|---------------------|
| **WiFi router** | All machines connect to a single router. If the router dies, inter-robot communication stops. The router is a **single point of failure** (SPOF). |
| **Map merging** | Only the laptop merges maps into `/map_merged`. The Jetsons do NOT have a global merged map — they only know their own local SLAM map. |
| **Visualisation** | RViz2 runs only on the laptop. There is no on-board visualisation on the Jetsons. |
| **DDS discovery** | Relies on multicast, which requires the router to support and forward multicast packets. |

### Where It Sits on the Spectrum

```
Fully Centralised              AUSRA (current)              Fully Decentralised
(1 brain, robots are dumb)         ↓                        (every robot is autonomous)
|────────────────────────|─────────●───────|────────────────────────|
                                   │
                    ┌──────────────┴──────────────┐
                    │  Centralised: WiFi router,   │
                    │    map merge on laptop only   │
                    │  Decentralised: on-board SLAM,│
                    │    on-board Nav2, DDS P2P      │
                    └───────────────────────────────┘
```

---

## 3. Why Each Robot Keeps Its Own Map

Even though the laptop merges all maps into `/map_merged`, **each Jetson retains its own SLAM map** (`/map` locally). This is a deliberate design choice:

1. **WiFi independence** — If the router fails or the robot moves out of WiFi range, it continues mapping and navigating using its local map.
2. **Computational load** — Map merging is CPU-intensive. Running it on the Jetson (already busy with SLAM + Nav2 + LiDAR processing) would degrade real-time navigation performance.
3. **Bandwidth** — The merged map doesn't need to flow back to the Jetsons. Each robot's navigation only needs its own local map.

### What This Means in Practice

- **Robot goes out of WiFi range**: Continues mapping and navigating autonomously. The laptop stops seeing updates from that robot, but the robot is unaffected.
- **WiFi comes back**: The relay_node immediately starts sending the latest map again. The base station incorporates it into the merged view.
- **Laptop crashes**: Both robots continue independently. They lose the merged view, but each robot has never depended on it.

---

## 4. Evolving Toward Full Decentralisation — On-Board Map Merge

### The Goal

Each Jetson subscribes to other robots' maps via DDS and runs its **own local map merge**. This way, every robot has a complete global map — not just its own SLAM output.

### What Would Change

```
[ CURRENT — Centralised Merge ]

Jetson 1 ──► /ausra_1/map ══DDS══► Laptop ──► map_merge ──► /map_merged
Jetson 2 ──► /ausra_2/map ══DDS══►

[ FUTURE — Decentralised Merge ]

Jetson 1 ──► /ausra_1/map ══DDS══► Jetson 2 (local merge) ──► /ausra_2/map_merged
                                    Laptop (merge for viz)  ──► /map_merged
Jetson 2 ──► /ausra_2/map ══DDS══► Jetson 1 (local merge) ──► /ausra_1/map_merged
                                    Laptop (merge for viz)  ──► /map_merged
```

### What You Would Need to Do

1. **Copy `ausra_comms_base` to each Jetson** (or create a lightweight variant without RViz2)
2. **Build `ausra_map_merge_HW` and `multirobot_map_merge` on each Jetson**
3. **Add a map merge launch to `hardware_with_comms.launch.py`** — starts `map_expansion_node` + `map_merge` after a delay, subscribing to other robots' maps
4. **Each Jetson subscribes to all `/ausra_X/map` topics** — DDS already delivers them automatically; just add the merge nodes

### Network Impact

Running map merge on each Jetson does **NOT** increase network traffic. The maps are already flowing over DDS from the relay_nodes. The only difference is that each Jetson now also *subscribes* to other robots' maps — but DDS handles this without extra bandwidth (it's the same multicast data).

However, there is one concern:

> **CPU load on Jetsons**: The Jetson Orin Nano is already running SLAM + Nav2 + LiDAR processing. Adding `map_expansion_node` + `multirobot_map_merge` will consume additional CPU/RAM. You should test this and monitor `htop` to ensure the Jetson can handle the extra load without dropping SLAM or Nav2 frames.

### Avoiding Conflicts Between Jetsons

| Concern | How to avoid |
|---------|-------------|
| **Topic namespace collisions** | Each robot's merge output should be namespaced: `/ausra_1/map_merged` vs `/ausra_2/map_merged`. The laptop's merge is `/map_merged`. No collision. |
| **DDS discovery storms** | With more subscribers, DDS sends more discovery packets. Use `discovery_rate: 0.05` (every 20 seconds) in the merge config to keep this minimal. |
| **Conflicting merge results** | Each robot merges independently. The merged maps may differ slightly due to timing differences in when each robot receives the latest map from others. This is normal and acceptable — each robot's local merged map is "good enough" for navigation. |
| **Duplicate map_expansion_nodes** | Each Jetson runs its own expansion nodes. The node names must be unique per machine (e.g., `map_expansion_ausra_1_on_jetson2`). Use the ROS 2 node name parameter to avoid conflicts. |

---

## 5. Improvements to the WiFi Approach

### 5.1 — Mesh Networking (Replace the Router)

**Problem:** The WiFi router is a single point of failure. If it dies, all inter-robot communication stops.

**Solution:** Use **WiFi ad-hoc (IBSS) mode** or **802.11s mesh** to create a direct robot-to-robot network without a router.

| Approach | Pros | Cons |
|----------|------|------|
| IBSS (ad-hoc) | Simple to set up, no router needed | Older standard, limited throughput, some drivers don't support it |
| 802.11s mesh | Better throughput, multi-hop routing, self-healing | More complex setup, requires kernel support |
| Mobile hotspot (phone) | Easiest "field" solution, no infrastructure needed | Phone battery drains, limited range |

For a **graduation project**, a mobile hotspot is the most practical. For a **production system**, 802.11s mesh would be the ideal choice.

### 5.2 — DDS Tuning

**Problem:** Default FastDDS settings are optimized for local networks, not constrained WiFi links.

**Improvements:**
- **Set `FASTRTPS_DEFAULT_PROFILES_FILE`** to configure buffer sizes, multicast TTL, and heartbeat periods
- **Use unicast discovery** instead of multicast — more reliable on routers that block multicast. List specific peer IPs in the DDS profile.
- **Reduce DDS history depth** — the default keeps 10 old messages in memory; for maps, keep only 1
- **Set QoS `keep_last(1)`** for large topics like maps — prevents memory buildup

### 5.3 — Map Compression

**Problem:** OccupancyGrid maps are sent as raw byte arrays over DDS. A 1000×1000 grid at 8 bits/cell = ~1 MB uncompressed.

**Solutions:**
- **Run-length encoding (RLE)** in the relay_node before publishing — most cells in a SLAM map are "unknown" (-1), so RLE achieves 5-10x compression
- **Use `sensor_msgs/CompressedImage`** as a transport format for map data
- **Publish only map diffs** — send only cells that changed since the last publish

### 5.4 — Peer-to-Peer Map Sharing

**Problem:** Currently, robots send maps to the laptop only. They don't share maps with each other.

**Solution:** Since DDS is inherently peer-to-peer, enabling this is trivial — any Jetson can subscribe to `/ausra_X/map` from any other Jetson. No code changes needed for the transport layer; just add the merge pipeline to each Jetson (see Section 4).

### 5.5 — Heartbeat-Based Fault Detection

**Problem:** If a robot disconnects, the base station has no way to distinguish "robot is dead" from "robot is exploring far away."

**Improvements:**
- **Timeout-based detection** — if no heartbeat is received for N seconds, mark the robot as "lost"
- **Signal strength monitoring** — subscribe to WiFi RSSI values and warn when a robot is at the edge of range
- **Graceful degradation** — if a robot is lost, the map merge should continue with the remaining robots' maps (currently works — the phantom node ensures this)

### 5.6 — Bandwidth Budgeting

**Problem:** On a congested WiFi link, large map messages can starve small heartbeat messages.

**Improvements:**
- **Priority queuing** — use DDS transport priority to give heartbeats higher priority than maps
- **Adaptive throttling** — increase `map_interval_sec` when WiFi quality drops (measure round-trip ping time)
- **Topic-level bandwidth caps** — configure FastDDS to limit per-topic throughput

### 5.7 — Security

**Problem:** Any device on the same WiFi network with `ROS_DOMAIN_ID=0` can inject messages or subscribe to robot data.

**Improvements:**
- **ROS 2 SROS2** — enable DDS security with encrypted communications and access control
- **Separate VLAN** — isolate robot traffic from other WiFi devices
- **DDS authentication** — require certificates for participant discovery

---

## 6. Summary

| Feature | Current State | Future State |
|---------|--------------|-------------|
| SLAM | ✅ On-board (decentralised) | ✅ No change |
| Navigation | ✅ On-board (decentralised) | ✅ No change |
| Map merge | ❌ Laptop only (centralised) | ✅ On-board + laptop (decentralised) |
| Transport | ✅ DDS P2P (decentralised) | ✅ No change |
| Network | ❌ WiFi router (centralised SPOF) | ✅ Mesh / IBSS (no SPOF) |
| Fault detection | ⚠️ Basic heartbeat only | ✅ Timeout + RSSI monitoring |
| Security | ❌ None | ✅ SROS2 encryption |
| Map compression | ❌ Raw bytes | ✅ RLE or diff-based |

The current system is a **solid foundation** that is production-ready for the 2-Jetson deployment. The improvements listed above are incremental enhancements that can be added one at a time without breaking the existing pipeline.

---

### Next Steps & Technical Guides
For concrete implementation details, code templates, and a 2-week roadmap for these features (including on-board map merging, collision avoidance, and frontier coordination), refer to the [Future Work Guide](file:///home/omen/ausra_ws/src/AUSRA-Autonomous-System-hardware_with_nav2/ausra_comms_base/docs/FUTURE_WORK.md).

