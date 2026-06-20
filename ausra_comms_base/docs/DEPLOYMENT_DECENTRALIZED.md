# Deployment Guide — Fully Decentralized Map Merge

> **Status: design / future deployment.** This describes how to run a map-merge
> instance on **every robot** as well as the base station, so the swarm keeps a
> merged map even if any single robot — or the laptop — dies. It is **not** the
> current default; the shipped setup is the semi-centralized one in
> [`DEPLOYMENT_2JETSONS.md`](DEPLOYMENT_2JETSONS.md). Read this end-to-end before
> changing any launch files — the code edits it depends on are listed in
> §7 and are **not yet applied**.

---

## 1. Why decentralize, and what actually changes

### 1.1 The two topologies side by side

**Semi-centralized (current).** Every Jetson compresses its local map and ships
`/ausra_X/map_compressed` over Zenoh. Only the **laptop** decompresses, expands,
and merges. The merged map lives in exactly one process. If the laptop dies, no
machine holds a merged map.

```
[Jetson 1] SLAM→relay→/ausra_1/map_compressed ─┐
                                                 ├─Zenoh→ [Laptop] decompress→expand→merge→/map_merged
[Jetson 2] SLAM→relay→/ausra_2/map_compressed ─┘
```

**Decentralized (this doc).** Every machine runs the full decompress → expand →
merge pipeline, so each holds its own merged map. The relay/compression layer is
unchanged — peers still need the compressed topic — but now *every* node both
publishes and consumes it.

```
[Jetson 1] SLAM→/ausra_1/map ─┬─ local merge → /ausra_1/map_merged
           relay→/ausra_1/map_compressed ═Zenoh═╗
[Jetson 2] SLAM→/ausra_2/map ─┬─ local merge → /ausra_2/map_merged
           relay→/ausra_2/map_compressed ═Zenoh═╣
[Laptop]   (no SLAM) ─────────┴─ merge → /map_merged   (+ RViz)
```

### 1.2 What makes the difference — the single most important idea

**A robot must never compress-then-decompress its own map for its own merger.**
Robot 1 already has raw `/ausra_1/map` on the DDS loopback — uncompressed,
in-process, free. It should feed *that* straight into its local expansion node,
and only run a decompressor for its **peers'** maps
(`/ausra_2/map_compressed`, …). `map_decompressor_node` already supports this via
the `ignore_robot` parameter, which skips creating a decompressor for the local
robot.

Net effect of that one rule:
- **Zero added compression cost.** The relay already compresses for the network;
  the local pipeline taps the raw topic that exists anyway.
- **Uniform expansion input.** Every expansion node — local and peer — subscribes
  to `/{name}/map`. Local SLAM serves its own; the decompressor reconstructs
  `/{peer}/map` for the rest. One code path, no special-casing.
- **One decompress per peer map per robot.** N robots × (N−1) decompress streams.
  For N=2 that is a single peer stream each — negligible (~1 KB zlib payload).

### 1.3 Difference table

| Concern | Semi-centralized (now) | Decentralized (target) |
|---|---|---|
| Merge engine runs on | Laptop only | **Every Jetson + laptop** |
| Packages on each Jetson | `ausra_comms` (+ `lidar_slam_pkg`) | `ausra_comms` + `ausra_map_merge_HW` + `multirobot_map_merge` |
| Decompressor scope | All robots (laptop) | **Peers only** (`ignore_robot:=<self>`) |
| Local map into merger | n/a on Jetson | Raw `/ausra_X/map` loopback (not compressed) |
| Zenoh `map_compressed` | Jetson pub / laptop sub | **Every node pub + sub** |
| Phantom grid | Not needed (2 cfg'd on laptop) | Needed per robot **if it can boot alone** |
| `robot_config` offsets | Passed to laptop once | **Passed to every Jetson** (each merger needs all offsets) |
| `merging_rate` vs map cadence | 1.0 Hz vs ~0.2 Hz map → ~5× idle CPU | Lower to match (see §6) |
| Failure domain | Laptop = single point of failure | No single point of failure |

**No base-station packages get copied to robots.** `ausra_comms_base` (RViz config,
`fake_robot_pub`, `start_base.sh`) stays laptop-only. The only genuinely new
artifact on the Jetson is one launch file with no RViz and no GUI deps (§3).

---

## 2. Package layout — keeping the Jetson self-contained

The decentralized pipeline needs three things on each robot. Two already build on
the Jetson; one must be added to the Jetson workspace.

| Component | Where it lives today | Jetson action |
|---|---|---|
| `relay_node`, `map_decompressor_node` | `ausra_comms` (ament_python) | already SCP'd — no change |
| `map_expansion_node` | `ausra_map_merge_HW` (ament_cmake C++) | **build on Jetson (ARM)** |
| `multirobot_map_merge` | `m-explore-ros2/map_merge` (ament_cmake C++) | **build on Jetson (ARM)** |

> **Why `map_decompressor_node` moves into `ausra_comms` and not a new package:**
> it is already a console_script in `ausra_comms/setup.py`, so the Jetson deploy
> stays one Python package for the comms layer. We deliberately do **not** copy
> `ausra_comms_base` — it pulls in RViz launch config and `fake_robot_pub` that a
> headless robot never needs.

> **Why the C++ packages must build on the Jetson:** `map_expansion_node` and
> `multirobot_map_merge` are compiled (ament_cmake). The laptop's x86 `install/`
> cannot be SCP'd to an ARM Jetson — you must `colcon build` them on each Jetson.
> This is the main "weight" of decentralizing, and the reason it is opt-in.

### One-time Jetson setup

```bash
# On each Jetson, into its workspace (e.g. ~/ausra_NM_ws/src/AUSRA-.../)
#   - ausra_comms              (already there)
#   - ausra_map_merge_HW       (copy the package source)
#   - m-explore-ros2/map_merge (copy the package source)
scp -r ausra_map_merge_HW            user@<JETSON_IP>:~/ausra_NM_ws/src/AUSRA-.../
scp -r m-explore-ros2/map_merge      user@<JETSON_IP>:~/ausra_NM_ws/src/AUSRA-.../m-explore-ros2/map_merge

# On the Jetson:
cd ~/ausra_NM_ws
colcon build --packages-select ausra_comms ausra_map_merge_HW multirobot_map_merge
source install/setup.bash
```

---

## 3. The new Jetson launch file (`hardware_decentralized.launch.py`)

This is the one new artifact. It lives in **`ausra_comms/launch/`** so the robot
deploy stays self-contained. It composes the existing single-robot bringup with a
local merge pipeline. Conceptually (full code is staged in §7):

```
hardware_decentralized.launch.py  (robot_name:=ausra_1  robot_config:="ausra_1:0:0 ausra_2:1.5:-2.0")
│
├─ IncludeLaunchDescription hardware_with_comms.launch.py     # SLAM + relay + Zenoh (unchanged)
│
└─ TimerAction(period=12s)   # let SLAM + relay come up first
   │
   ├─ map_decompressor_node                                   # PEERS ONLY
   │     robots:       [all from robot_config]
   │     ignore_robot: <robot_name>                           # ← do NOT decompress own map
   │
   ├─ map_expansion_<name> per robot in robot_config          # input_topic = /<name>/map
   │     local robot  → reads raw /<robot_name>/map  (loopback)
   │     peer robots  → read decompressed /<peer>/map
   │     output_frame_id: map      robot_offset_x/y: from robot_config
   │
   └─ multirobot_map_merge   namespace=/<robot_name>          # → /<robot_name>/map_merged
         params: map_merge_HW_params.yaml + dynamic_init_poses(all=0.0)
```

Key points that mirror existing, proven code:
- The expansion-node parameters are **identical** to
  `ausra_comms_base/launch/map_merge.launch.py` (canvas 1000×1000, origin
  −25/−25, `output_frame_id: map`, `init_pose_* = 0.0`). Decentralizing changes
  *where* it runs, not the spatial math.
- The merger is pushed into `namespace=/<robot_name>` and `merged_map_topic` is
  **relative** (`map_merged`, not `/map_merged`) so each robot publishes
  `/<robot_name>/map_merged` and they never collide. This matches the
  wildcard-namespace design already documented in
  [`how_i_can_make_map_merge_on_each_robot.md`](../../ausra_map_merge_HW/docs/how_i_can_make_map_merge_on_each_robot.md).

---

## 4. Propagating `robot_config` to the Jetsons

### 4.1 Do I pass it to every Jetson? — Yes.

In the centralized setup only the laptop merges, so only the laptop needs the full
offset list. **Decentralized, every Jetson runs its own merger and therefore needs
the offsets of *all* robots**, not just itself. The local merger must know where
each peer's canvas sits to place it correctly.

```bash
# Same string on BOTH robots and the laptop. robot_name differs; robot_config does not.
# Robot 1:
ros2 launch ausra_comms hardware_decentralized.launch.py robot_name:=ausra_1
    robot_config:="ausra_1:0.0:0.0 ausra_2:1.5:-2.0"

# Robot 2:
ros2 launch ausra_comms hardware_decentralized.launch.py \
    robot_name:=ausra_2 \
    robot_config:="ausra_1:0.0:0.0 ausra_2:1.5:-2.0"

# Laptop (base station, optional in decentralized mode — RViz/aggregation only):
./ausra_comms_base/scripts/start_base.sh \
    robot_config:="ausra_1:0.0:0.0 ausra_2:1.5:-2.0"
```

The offsets are **tape-measured physical spawn positions**, identical on every
machine. The parser is the same `name:x:y` space-separated format already used by
both `map_merge.launch.py` and `map_merge_hw.launch.py`, so behaviour (whitespace
tolerance, malformed-entry skip, empty-string safe) is unchanged — see the
[architecture review](../../ausra_map_merge_HW/docs/Multi_robot/architecture_review.md).

> **Critical invariant (unchanged):** `init_pose_*` stays `0.0` for every robot in
> every merger. `map_expansion_node` already bakes the offset into canvas pixels;
> setting `init_pose_*` to the spawn coords double-shifts. The launch file injects
> them as `0.0` dynamically — do not move offsets into the YAML.

### 4.2 What each robot uses its offsets for

`robot_config` does double duty on every Jetson:
1. **`robot_offset_x/y`** for *its own* expansion node (where its canvas sits).
2. **The full set** drives one expansion node per robot + the `init_pose=0.0`
   keys, so the local merger discovers and aligns every peer.

---

## 5. Zenoh allowlist — already correct on the Jetson side

Decentralized comms need every robot to **publish and subscribe**
`/ausra_*/map_compressed`. The Jetson config already does both:

```json5
// ausra_comms/config/zenoh_bridge_jetson.json5  (already correct)
allow: {
  publishers:  ["/ausra_.*/map_compressed", "/ausra_.*/heartbeat"],
  subscribers: ["/ausra_.*/map_compressed", "/ausra_.*/heartbeat"],
}
```

So robot 1's bridge already forwards robot 2's `map_compressed` into robot 1's DDS
graph, where its decompressor picks it up. **No Zenoh change is required for the
robots.** The laptop config only subscribes (it never publishes a map), which is
also still correct. If you later add a topic to the cross-WiFi set, edit **both**
json5 files (the two-files-in-sync rule from `CLAUDE.md` §8 still holds).

> Heartbeats become more useful here: each robot can watch `/ausra_*/heartbeat` to
> know which peers are live and (optionally) prune dead robots from its merge view.

---

## 6. Performance & bandwidth tuning

These apply to the decentralized layout specifically, where CPU now matters on the
Jetson (it is doing SLAM **and** merging).

### 6.1 Match merge frequency to map cadence (biggest CPU win)

`relay_node` ships a map at most every `map_interval_sec` (default **5 s** = 0.2 Hz),
and adaptive throttling can stretch that to 15–30 s. But the merger runs at
`merging_rate: 1.0` Hz and the expansion heartbeat at `publish_rate_hz: 1.0`.
That is up to **5× redundant** recompositing of identical canvases on a
CPU-constrained Jetson.

```yaml
# map_merge_HW_params.yaml  — for on-robot merging
merging_rate:   0.25     # ~one merge per relayed map, not 5
discovery_rate: 0.1      # peers join slowly; no need to scan 20×/s
```

Keep `publish_rate_hz: 1.0` on the expansion node **only** if the merger relies on
a fresh heartbeat to stay alive; otherwise drop it to `0.5`. The phantom/heartbeat
guarantee (a valid canvas always present so the merger can't segfault on <2 grids)
is preserved as long as the rate stays > 0.

### 6.2 Don't double-handle the local map

Covered in §1.2 — the local map enters the merger as raw loopback, never through
compress→Zenoh→decompress. This is the single biggest bandwidth and CPU saving and
is enforced by `ignore_robot:=<self>`.

### 6.3 QoS queue depth

The compression fix already set `/ausra_*/map_compressed` to
`TRANSIENT_LOCAL + RELIABLE, depth 2` on both relay and decompressor (CLAUDE.md
§9). Depth 2 is correct here: maps are large and infrequent, so a deep queue just
buffers stale 1 MB grids. Leave it at 2; do **not** raise it to absorb jitter —
`TRANSIENT_LOCAL` already redelivers the latest sample to late joiners.

### 6.4 Compression CPU on the Jetson

`zlib.compress(level=6)` on a 1 MB grid is cheap (sparse maps hit >99% in a few
ms), and `relay_node` now runs it in a `MultiThreadedExecutor` so it never blocks
heartbeats. If a Jetson is CPU-bound during heavy SLAM, drop the relay to
`level=4` — near-identical ratio on mostly-unknown grids, less CPU. Do **not** go
to `level=9`; the extra CPU buys almost nothing on −1-dominated data.

### 6.5 `map_expansion_node` is already optimal

The partial-reset (only clears the cells written last callback, ~10k–100k ops vs a
1M-cell `std::fill`) and the row-level boundary skip are already in the C++ node.
No change needed; just be aware each robot now runs N of these.

---

## 7. Code changes this depends on (NOT yet applied)

This doc is design-ahead. Before running the above, these edits must land:

1. **New** `ausra_comms/launch/hardware_decentralized.launch.py` (§3).
2. **`ausra_comms/setup.py`** — install `ausra_map_merge_HW` / `multirobot_map_merge`
   are separate packages, so only ensure the new launch file is globbed (the
   `launch/*.launch.py` glob already covers it).
3. **`map_merge_HW_params.yaml`** — relative `merged_map_topic: map_merged` and the
   tuned `merging_rate` (§6.1). Keep a laptop copy with `/map_merged` if the base
   station still aggregates.
4. **Jetson build** of `ausra_map_merge_HW` + `multirobot_map_merge` (§2).
5. Confirm `map_decompressor_node`'s `ignore_robot` path is exercised
   (already implemented; just wired via the new launch file).

No changes to `relay_node.py`, the Zenoh json5 files, or `map_expansion_node.cpp`
are required — the decentralized topology reuses them as-is.

---

## 8. Failure modes & verification

| Event | Decentralized behaviour |
|---|---|
| Laptop dies | Both robots keep merging locally → `/ausra_X/map_merged` intact |
| Robot 2 dies | Robot 1's decompressor stops getting `/ausra_2/map_compressed`; its expansion node freezes robot 2's last canvas (ghost-map, by design — CLAUDE.md §5) and keeps merging |
| Robot 2 joins late | Its `map_compressed` appears on Zenoh; robot 1's decompressor + expansion node pick it up next discovery tick |
| Robot boots alone | Local phantom/heartbeat canvas keeps its merger from segfaulting on <2 grids |

Verify on any robot:

```bash
# Peer's compressed map is arriving over Zenoh:
ros2 topic hz /ausra_2/map_compressed          # on robot 1

# Decompressor reconstructed the peer's grid:
ros2 topic info /ausra_2/map -v                 # durability must read TRANSIENT_LOCAL

# Local merged map exists and is namespaced:
ros2 topic echo /ausra_1/map_merged --once | head

# Own map is NOT being decompressed (loopback, not via map_compressed):
ros2 node list | grep map_decompressor          # logs "Ignoring local robot: ausra_1"
```

---

## 9. Related docs

- [`DEPLOYMENT_2JETSONS.md`](DEPLOYMENT_2JETSONS.md) — the current semi-centralized runbook.
- [`how_i_can_make_map_merge_on_each_robot.md`](../../ausra_map_merge_HW/docs/how_i_can_make_map_merge_on_each_robot.md) — namespace/wildcard rationale this builds on.
- [`architecture_review.md`](../../ausra_map_merge_HW/docs/Multi_robot/architecture_review.md) — `robot_config` parsing + `init_pose` zeroing validation.
- [`ZENOH_GUIDE.md`](ZENOH_GUIDE.md) — allowlist and transport rules.
- `CLAUDE.md` §2 (QoS), §3 (offset baking), §4 (phantom), §5 (ghost map), §8 (Zenoh), §9 (compression).
