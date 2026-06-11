# Interception pipeline — architecture & analysis

This document records the design of the drone-to-drone interception pipeline, an
analysis of the **old d2dtracker/iHunter framework** it was ported from, the
**state-machine** orchestration, the **old-vs-new differences**, and the
infrastructure decisions made during the port. It is the single reference for
how the system is wired and why.

- Toolchain: **ROS 2 Jazzy · Gazebo Harmonic · PX4 v1.15 · FastRTPS RMW**.
- Workspace: `drone_interception_ws/ros2_ws/src` (custom work) + in-tree
  `drone_interception_ws/PX4-Autopilot` (SITL).

---

## 1. The old d2dtracker / iHunter framework (what we ported from)

The original framework (in `d2dtracker-…/d2dtracker/ros2_ws/src`) is a **cascade
with the state machine sitting in the middle as the orchestrator**:

```
camera ─▶ drone_detector ─▶ multi_target_kf ─▶ drone_path_predictor (GRU)
                                                        │ predicted path
                                                        ▼
        arm/mode services ◀── [ d2dtracker_states FSM ] ──▶ desired waypoint Path
        (AUTO.TAKEOFF→OFFBOARD                                   │
         →AUTO.RTL→AUTO.LAND)                                    ▼
                                          trajectory_generation (12-state MPC)
                                                                 │ MultiDOFJointTrajectory
                                                                 ▼
                                  interceptor_offboard_control (extracts velocity)
                                                                 │ PositionTarget (VELOCITY)
                                                                 ▼
                                                          mavros ─▶ PX4
```

Key logic:
- The **FSM is the brain**: consumes KF detections + the predicted path, decides
  the mission phase, **emits the reference** (a waypoint `Path`) the MPC tracks,
  and owns **arming + flight-mode switching**.
- Phase sequence: `Idle → Arm → Takeoff → Surveillance → Pursuit → Attack →
  Return → Land`. It does a real **`AUTO.TAKEOFF` to altitude first**, *then*
  `OFFBOARD` to pursue. Pursuit→Attack when range ≤ `attack_distance` (0.5 m)
  using a *lagged* predicted target pose; Attack→Return on intercept → `AUTO.RTL`.
- Final command to PX4 is a **velocity `PositionTarget`** (an offboard-control
  node extracts the MPC trajectory's velocity + `yaw=atan2(vy,vx)`).

### State-machine repo (`d2dtracker_states`)
Two interchangeable mission controllers over the same MAVROS interface:
- **FSM** (`ros_node.py`, `state_machine` entry) — the 8-state machine above
  (ported ~verbatim into the current workspace).
- **BT** (`bt/`, `bt_node` entry) — a py_trees reactive priority `Selector`:
  `PostCapture(Captured?→RTL) ▸ Attack(Airborne?+WithinAttackDistance?→Attack) ▸
  Pursue(Airborne?+TargetDetected?→Pursue) ▸ TakeoffSearch(EnsureOffboardArmed→
  Takeoff→Search)`. Its bridge reads `intercept_point` (from
  `d2dtracker_interception`) + detections + odom and **streams PositionTarget
  setpoints directly** to `setpoint_raw/local` — it is orchestrator *and*
  controller (no MPC/control cascade).

---

## 2. The new pluggable pipeline (this workspace)

Every stage of the old cascade is re-implemented as its **own package with a
stable message contract and swappable algorithm backends** selected by a
`backend:=`/`model_type:=` parameter. Two pluggability layers:
- **Inter-stage contracts** — stable topics + `interception_msgs` types, so whole
  stage nodes (even Python↔C++) swap at launch.
- **Intra-stage backends** — a registry/factory per stage; add an algorithm = one
  subclass + one registration line.

```
camera/gt ─▶ [detection] ─▶ [estimation/KF] ─▶ [prediction] ─▶ [planning] ─▶ [MPC] ─▶ [control] ─▶ PX4
             detector=        estimator=          predictor=       planner=      mpc=      controller=
             ground_truth|    const_vel|          const_vel|       rendezvous|   pass-     px4_setpoint|
             depth|yolo       const_accel         const_accel|     tail_chase|   through|  pn_velocity|
                                                  poly|gru         pn_lead|      mpc_12    se3
                                                                   head_on       state
```

| Package | Build | Contract out | Backends |
|---|---|---|---|
| `interception_msgs` | ament_cmake | `Detection[Array]`, `TargetEstimate[Array]`, `State`, `StateTrajectory` | — |
| `interception_detection` | ament_python | `DetectionArray` + `PoseArray` | `ground_truth`·`depth`·`yolo` |
| `interception_estimation` | ament_cmake (C++) | `TargetEstimateArray` + `PoseArray` | `const_vel`·`const_accel` (`MotionModel*` + `model_type:=`) |
| `interception_prediction` | ament_python | `StateTrajectory` + `Path` | `const_vel`·`const_accel`·`poly`·`gru` |
| `interception_planning` | ament_python | `PoseStamped intercept_point` + `StateTrajectory reference` + `Path` | `rendezvous`·`tail_chase`·`pn_lead`·`head_on` |
| `interception_mpc` | ament_cmake (C++) | `StateTrajectory command_trajectory` | `passthrough`·`mpc_12state` (OSQP via osqp-eigen) |
| `interception_control` | ament_cmake (C++) | mavros setpoints | `px4_setpoint`·`pn_velocity`·`se3` |

Topic wiring (interceptor namespace):
`detection_node/detections_poses → kf/good_tracks(_pose_array) → predicted_trajectory
→ reference + intercept_point → command_trajectory → mavros/setpoint_raw/local`.

Backends reuse the original pure-math where possible: planning imports
`d2dtracker_interception` (`find_optimal_intersection`, `pn_velocity_command`,
`reference_trajectory`); the MPC and SE3 controller are verbatim C++ ports of
`trajectory_generation` / `mav_controllers_ros`.

---

## 3. Orchestration head (selectable)

The old FSM did **two** jobs: (a) phase sequencing + arm/mode management, and
(b) producing the reference. In the new pipeline the **planning stage produces
the reference (b)**, so the orchestration head only needs **(a)** — and, because
**PX4 ignores OFFBOARD setpoints unless it is in OFFBOARD mode**, the head can be
pure arm+mode+phase sequencing: the cascade streams setpoints the whole time and
the head just switches modes.

The head is selected with `pipeline.launch.py orchestrator:=`:

| `orchestrator:=` | what runs | flies via |
|---|---|---|
| `fsm` *(default)* | `pipeline_fsm` (Idle→Arm→Takeoff `AUTO.TAKEOFF`→Pursue `OFFBOARD`→Attack→Return `AUTO.RTL`) **+ MPC→control cascade** | the cascade's control stage |
| `offboard` | `offboard_manager` (arm+OFFBOARD only) **+ cascade** | the cascade |
| `bt` | `d2dtracker_states` Behaviour Tree (MPC+control **not** started) | the BT (direct setpoints from `intercept_point`) |
| `none` | cascade only, nothing arms/flies | — (topic inspection / RViz) |
| `rl` | reserved for a `d2dtracker_rl` policy head | (future) |

`pipeline_fsm` (`drone_interception_sim/pipeline_fsm.py`) is the FSM head wired
onto the cascade: it transitions on altitude / target detection
(`kf/good_tracks_pose_array`) / interceptor→target range, and sequences PX4
modes only — the planning→MPC→control cascade supplies the OFFBOARD setpoints.
This restores the old framework's structure (state machine on top of the
cascade) with the new, pluggable stages underneath.

Adding an orchestrator = a node that manages arm/mode/phase + one `orchestrator:=`
branch in `pipeline.launch.py` (and, if it drives directly like the BT, gate off
the cascade by not starting MPC/control for that branch).

---

## 4. Old vs new — differences

| Aspect | Old framework | New pipeline |
|---|---|---|
| Reference source | FSM emits phase-aware waypoint `Path` | `interception_planning` emits the intercept reference continuously |
| Orchestration | FSM in the data path, sequences phases + reference | `pipeline_fsm` head sequences **modes only**; planning owns the reference |
| State machine | central, bespoke | reused as a *selectable head* (`fsm`/`bt`); `none`/`offboard`/`rl` also selectable |
| MPC→control link | MPC `MultiDOFJointTrajectory` → offboard node | MPC `StateTrajectory` → `interception_control` |
| Setpoint to PX4 | velocity `PositionTarget` | position `PositionTarget` (`px4_setpoint`); `se3`→`AttitudeTarget` |
| Default predictor | GRU (trained weights) | `const_vel` (GRU ported, falls back without weights) |
| Default controller | geometric/SE3 attitude | `px4_setpoint` (SE3 ported, unvalidated in flight) |
| Messages | `custom_trajectory_msgs` + `multi_target_kf` | unified `interception_msgs` |
| Transport | (Humble/zenoh era) | **FastRTPS** (zenoh drops high-rate `/clock`+odom here) |
| Pluggability | fixed wiring | per-stage `backend:=`, swap one without touching others |

---

## 5. Infrastructure decisions (port notes)

- **RMW = FastRTPS, not zenoh.** On this machine zenoh reliably delivers only
  low-rate topics (e.g. `mavros/state`) but **drops high-rate ones** (odom
  @30 Hz, `/clock` @200 Hz), breaking sim-time and the perception chain. The
  launch can't force the RMW for its included mavros nodes (they inherit the
  shell env), so **`scripts/run_sim.sh` exports `rmw_fastrtps_cpp` before
  `ros2 launch`** — use it, and export the same RMW + `ROS_DOMAIN_ID` in every
  terminal.
- **osqp-eigen vendored.** `interception_mpc` needs `OsqpEigen`; osqp-eigen 0.8.0
  is vendored into `src/osqp-eigen` and builds against the system osqp 0.6.2
  (version-compatible). `osqp` itself comes from `ros_jazzy_osqp_vendor`.
- **Sim self-clean.** Repeated relaunches used to accumulate zombie nodes
  (multiple gz servers + `/clock` bridges → "jump back in time"). `interceptor.launch.py`'s
  `_kill_stale_sim` and `scripts/stop_sim.sh` now kill the sim's **ROS nodes +
  prior launch groups**, not just px4/gz by partition. `pipeline.launch.py` is the
  single entry point for the stages (no hand-launched duplicates).
- **KF C++ fixes** (`interception_estimation`): `SensorDataQoS` on the
  measurement subscription (FastRTPS dropped the default reliable C++ sub);
  predict-to-measurement-time in `updateTracks` (the `@todo`) + finite-difference
  velocity seeding of new tracks — together these stop track churn so
  `good_tracks` confirms (validated: `n`→24, ~one persistent track, velocity ≈
  target speed).
- **KF estimate accuracy / time base** (the "estimate is ~6 m off" regression).
  Two coupled causes, both fixed:
  1. **Clock.** The KF stamps each measurement with `this->now()` at receive
     (`tracker_ros.h`), which is only valid on **sim time** — there the Gazebo
     clock barely advances during ROS transport, so receive-time ≈ sample-time.
     On wall clock that gap is real latency+jitter and the velocity/extrapolation
     drift. So the **cascade runs `use_sim_time:=true`** (heads/viz stay wall).
     Do *not* forward the odom's `header.stamp` into the KF — in this sim it sits
     on a different clock and yields a ~−338 s `dt` that breaks the filter.
  2. **Process noise.** `sigma_a` defaulted to **1.0**, 10× below the proven
     `kf_param.yaml` value of **10.0**. At 1.0 the constant-velocity filter is
     over-smoothed and cannot follow a turning target, so the estimate lags onto
     a larger circle. Set to **10.0** (plus `dt_pred=0.02`, `N_meas=5`,
     `l_threshold=-100`, matching SMART-TRACK). Result: estimate↔actual gap on a
     circling target dropped from **~6 m to ~0.06 m**.
- **Adopted from SMART-TRACK** (`SMART-TRACK/smart_track`, the reference vision-KF
  framework): the sim-time convention + the tuned KF params above. The same
  `multi_target_kf` core (state buffer for delayed measurements, log-likelihood +
  Hungarian association, multi-gate `V_certain`/`N_meas`/timeout confirmation,
  runtime param retuning) is already shared. **Not yet ported** (future, camera
  path): SMART-TRACK's *KF→detection feedback loop* (use the KF track + its
  covariance to define a depth-image search ROI when YOLO misses) and the
  `ApproximateTimeSynchronizer(slop=0.1)` detection/depth fusion.
- **Camera** (`depth`/`yolo` detectors) is blocked by an EGL/`gimbal_small_3d`
  issue; `ground_truth` is the default detector so the whole cascade runs
  camera-free.

---

## 6. Running it

```bash
# 1. sim (one terminal) — exports FastRTPS:
src/drone_interception_sim/scripts/run_sim.sh gpu:=false
#    every other terminal:
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=77

# 2. full pipeline, FSM head + cascade (the interceptor takes off then pursues):
ros2 launch drone_interception_sim pipeline.launch.py orchestrator:=fsm metrics:=true
#    swap a stage, nothing else changes:
ros2 launch drone_interception_sim pipeline.launch.py predictor:=poly controller:=se3
#    behaviour-tree head (direct drive, no MPC/control):
ros2 launch drone_interception_sim pipeline.launch.py orchestrator:=bt
#    pure cascade for tuning (arm+offboard only / nothing):
ros2 launch drone_interception_sim pipeline.launch.py orchestrator:=offboard
ros2 launch drone_interception_sim pipeline.launch.py orchestrator:=none

# inspect any stage:
ros2 topic echo /interceptor/kf/good_tracks --once
ros2 topic echo /interceptor/intercept_point --once
ros2 topic echo /interceptor/mission/phase          # FSM head phase
ros2 topic hz   /interceptor/mavros/setpoint_raw/local
```

---

## 7. Status & known gaps

- **CLOSED-LOOP CAPTURE VALIDATED** with `orchestrator:=fsm` (FSM head + the full
  planning→MPC→control cascade, `px4_setpoint` position control). Full mission:
  `Idle→Arm→Takeoff→Pursue→Attack→CAPTURE (0.82 m)→Return→Done` — the interceptor
  takes off (`AUTO.TAKEOFF`), engages OFFBOARD, the cascade flies it from spawn to
  the target (range 12 m → 0.82 m over ~30 s), capture latches inside the 1.0 m
  radius, then `AUTO.RTL` home. The earlier "armed but hovering" was *entirely*
  the missing takeoff phasing, not a controller problem — position setpoints fly
  to the target fine once airborne.
- Two fixes that unlocked it: the **orchestration heads run on wall clock**
  (`use_sim_time:=False`) so their mission timer keeps ticking even when
  FastRTPS's `/clock` delivery stalls (the whole pipeline now defaults to wall
  clock for the same robustness); and the FSM **leaves `AUTO.RTL`/`AUTO.LAND`
  before re-arming**.
- **Per-stage outputs verified** along the way: `good_tracks` confirms; planning
  gives a reachable intercept; MPC gives a feasible trajectory; control emits
  valid setpoints; the FSM head arms + sequences modes + latches capture.
- **Prediction = OLD-implementation parity (the trained GRU), now the default.**
  The old deployment predicted with two encoder-decoder GRUs (position +
  velocity, 256x5, whitening, 2.0 s in @ 0.1 s → 1.0 s out) and published the
  **velocity-GRU** path (`use_velocity_prediction: true`): finite-diff the input
  window, predict the future velocity sequence, integrate from the last
  position. The port initially differed three ways — no weights wired (silent
  const_vel fallback), the **whitening transform inverted** (`@Linv.T` in /
  `@L.T` out instead of the old `@L.T` in / `@inv(L).T` out), and only the
  position GRU implemented. All fixed: the trained weights + stats ship in
  `interception_prediction/models/`, both GRUs load (CUDA), the velocity path is
  the default, and `predictor:=gru` is the pipeline default (`history_len` 150
  so the 2 s input window fills). Measured on the circling target: offline
  RMSE@1s **0.044 m (GRU-vel)** vs 0.270 m (const_vel) vs 0.568 m (GRU-pos —
  why the old config chose the velocity path); live overall RMSE **0.19 m**
  (0.26 m at the 1 s horizon), and the mission captured at **0.67 m** (best
  run). `const_vel`/`const_accel`/`poly` remain selectable.
- **Open items:** `interception_metrics` capture-CSV stayed empty in the run
  (its own capture detector uses `interceptor_spawn`/`target_spawn` params — check
  those offsets vs the world frame; the FSM's range is the authoritative capture
  signal). `se3` controller + `pn_velocity` are built but unvalidated in flight.
  `rl` orchestrator is a reserved slot (would wrap a `d2dtracker_rl` policy).
- **Operational note:** launch the pipeline only via `pipeline.launch.py` (single
  entry); hand-launching individual stages across runs leaves orphan nodes
  (duplicate predictors/planners publishing to the same topics) that corrupt the
  estimate — the cause of several earlier red herrings.
