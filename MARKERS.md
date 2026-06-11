# RViz markers reference

Everything the interception pipeline draws in RViz, and **how to actually see it**.

## TL;DR — why markers may not appear

The markers are published by the `pipeline_viz` node **only while the pipeline is
running**, as a single `MarkerArray` in the `map` frame. **They display in the
sim's own RViz** — `interception.launch.py` launches RViz with
`rviz/interception.rviz`, which includes the *Pipeline (estimate/prediction/plan)*
display; there is no separate pipeline RViz. To see them you need **all** of:

1. The **pipeline running** with `viz:=true` (it is on by default) — markers stop
   the instant the pipeline does.
2. The **sim's RViz** (launched with the sim; `use_rviz:=true` default). If you
   open RViz manually, do it in a shell with `export
   RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=77` — an RViz from another
   project or domain shows nothing of ours.
3. RViz **Fixed Frame = `map`** (the shipped config sets this).
4. A **MarkerArray display** subscribed to `/interceptor/interception/markers`
   with **Reliability = Reliable** (the shipped config sets this).
5. The sim's RViz **loads the config at startup** — if it was already open before
   this display was added to `interception.rviz`, restart the sim (or add the
   display manually: *Add → By topic → /interceptor/interception/markers*).

One-command check that the markers are actually flowing:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=77
ros2 topic hz /interceptor/interception/markers          # ~10 Hz when the pipeline runs
ros2 topic echo /interceptor/interception/markers --once | grep ns:   # the 6 namespaces
```
If `hz` says nothing → the pipeline/viz isn't running. If it ticks but RViz is
blank → it's an RViz-side issue (wrong window/domain, Fixed Frame, or the display
isn't added). See **Troubleshooting** at the bottom.

---

## The unified MarkerArray — `/interceptor/interception/markers`

Published by `pipeline_viz` at 10 Hz, all in the **`map`** frame. One RViz
`MarkerArray` display shows every item below. Each sphere also has a floating text
label (a second marker, `id:1`, in the same namespace).

| namespace (`ns`) | shape | colour | what it is | source topic | drawn when |
|---|---|---|---|---|---|
| `target_actual` | SPHERE 0.6 m | 🟢 green | target's **true** pose (`target_odom + target_spawn`) | `/target/mavros/local_position/odom` | target odom present |
| `target_estimate` | SPHERE 0.5 m | 🟡 yellow | target pose **estimated** by detection→KF | `/interceptor/kf/good_tracks_pose_array` | KF has a confirmed track |
| `interceptor` | SPHERE 0.6 m | 🔵 blue | interceptor pose (`interceptor_odom + interceptor_spawn`) | `/interceptor/mavros/local_position/odom` | interceptor odom present |
| `intercept_point` | SPHERE 0.4 m | 🔴 red | planned rendezvous point | `/interceptor/intercept_point` | planner publishing |
| `predicted_path` | LINE_STRIP | 🟦 cyan | target's **predicted future** trajectory | `/interceptor/predicted_path` | prediction has ≥2 pts |
| `planned_path` | LINE_STRIP | 🟣 magenta | interceptor's **planned approach** (interceptor → reference) | `/interceptor/reference_path` (+ interceptor pos) | reference + interceptor present |
| `estimation_error` | TEXT | green / red | live `est err X m` readout = ‖actual − estimate‖ (green < 0.5 m, red above) | computed from the two above | actual + estimate present |

Reading it: the **`est err` text is the live estimation error** (the distance
between the green *actual* and yellow *estimate* spheres) — healthy tracking
reads a green `est err 0.0x m` with the two spheres overlapping. **Do not judge
the estimator by the cyan line or the red sphere**: the cyan *predicted* path and
the red *intercept point* are deliberately ahead of the target (they forecast and
lead it) — them being away from the target is correct behaviour, not estimation
error. The blue sphere chases toward the red intercept point along the magenta
planned path.

Marker details (from `pipeline_viz.py`): frame `map` (param `world_frame`); world
positions computed as `odom + spawn` (params `interceptor_spawn` `[0,0,0]`,
`target_spawn` `[10,0,0]`) — the same convention the `ground_truth` detector uses,
so no TF wiring is required. Publisher QoS: Reliable, depth 1.

---

## Real drone bodies (optional) — `/interceptor/markers`, `/target/markers`

The `drone_markers` node draws each PX4 craft as a quadcopter mesh
(`MarkerArray`, `MESH_RESOURCE`/`CUBE`) **parented to the drone's `base_link`
frame**, so they need the sim's `map → base_link` TF to place correctly.

- These are **not** started by `pipeline.launch.py` — they come up with
  `interception.launch.py` (the sim). The sim's `interception.rviz` config
  includes the two displays already (they simply stay empty if `drone_markers`
  isn't running).

---

## Native-display alternatives (no viz node)

If you prefer RViz's built-in displays over the unified MarkerArray, every item is
also available as a standard message (all in `map` except the odometries):

| item | RViz display | topic | QoS |
|---|---|---|---|
| interceptor pose | Odometry | `/interceptor/mavros/local_position/odom` | **Best Effort** |
| target pose (raw) | Odometry | `/target/mavros/local_position/odom` | **Best Effort** |
| target estimate | PoseArray | `/interceptor/kf/good_tracks_pose_array` | Best Effort |
| detections | PoseArray | `/interceptor/detection_node/detections_poses` | Best Effort |
| predicted trajectory | Path | `/interceptor/predicted_path` | Reliable |
| planned path | Path | `/interceptor/reference_path` | Reliable |
| intercept point | Pose | `/interceptor/intercept_point` | Reliable |

> The two mavros odometries are **Best Effort** — set the Odometry display's
> Reliability to *Best Effort* or it shows nothing. The odom topics are in each
> craft's own EKF frame, so the raw target Odometry needs the `map→target/odom`
> TF to sit at the true world position (this is exactly why the viz node
> republishes them as `map`-frame spheres instead).

---

## How to see them — step by step

```bash
# 1. sim (terminal A) — exports FastRTPS; this launches THE RViz (interception.rviz):
cd ~/drone_interception_ws/ros2_ws && source install/setup.bash && export ROS_DOMAIN_ID=77
src/drone_interception_sim/scripts/run_sim.sh gpu:=false

# 2. pipeline (terminal B) — its markers appear in the sim's RViz automatically:
source install/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=77
ros2 launch drone_interception_sim pipeline.launch.py orchestrator:=fsm
```

The sim's RViz (Fixed Frame `map`) already contains the *Pipeline
(estimate/prediction/plan)* MarkerArray display. Watch the interceptor arm, take
off, and the blue sphere fly to the green/yellow target along the magenta path,
with the green `est err 0.0x m` readout above the estimate.

If you must add the display **by hand** in some other RViz:
`Add → By topic → /interceptor/interception/markers → MarkerArray`, then set the
display's *Topic → Reliability Policy* to **Reliable**, and *Global Options →
Fixed Frame* to **map**.

---

## Troubleshooting "I can't see the markers"

| Symptom | Cause | Fix |
|---|---|---|
| `ros2 topic hz /interceptor/interception/markers` prints nothing | pipeline/viz not running | launch `pipeline.launch.py` (with `viz:=true`, default); markers exist only while it runs |
| `hz` ticks but RViz is blank | looking at the **wrong RViz** (another window/project, or a different `ROS_DOMAIN_ID`) | close all RViz, open one with `export ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_fastrtps_cpp` first |
| RViz says *Fixed Frame [map] does not exist* | no `map` TF (sim not up) or wrong frame set | start the sim; set Fixed Frame to **map** |
| MarkerArray display exists but empty | QoS mismatch | set the display's Reliability to **Reliable** |
| Only some markers show | that stage has no data yet | e.g. `target_estimate` needs the KF to confirm; `planned_path`/`intercept_point` need the planner; check each source topic with `ros2 topic hz` |
| Odometry display empty | mavros odom is **Best Effort** | set the Odometry display Reliability to **Best Effort** |
| Markers in the wrong place | spawn offsets differ from `[0,0,0]`/`[10,0,0]` | pass matching `interceptor_spawn`/`target_spawn` to the viz node |
| Several RViz windows / sims at once | leftover instances from prior runs | `scripts/stop_sim.sh`; launch only via `pipeline.launch.py` |

See `DEMO.md` for the full run walkthrough and `ARCHITECTURE.md` for the pipeline
design.
