# Interception benchmark

Capture benchmark of every interception method that flies in the two-drone
scene, 10 trials per method and target behaviour. Run 30–31 Oct 2026 with
`scripts/run_benchmark.py`; on 1 Nov textbook PN (`pn_int`) and the last 5
cells were added and the SE(3) controller re-flown with its corrected thrust
model. Data, traces and figures are in
[`benchmarks/2026-10-30_pipeline_vs_pn/`](benchmarks/2026-10-30_pipeline_vs_pn/).
It replaces the June 2026 single-run table (kept at the end for history).

## Setup

- **Scene:** `interception.launch.py`, headless. Interceptor `x500_d435`
  (2.466 kg) spawns at (0, 0); target `x3_uav` at (10, 0) flies a 6 m circle
  at 0.4 rad/s (2.4 m/s) centred 10 m further on, at about 4 m altitude.
- **Scenarios:**
  - `circle`: the scripted circle.
  - `evade`: the same circle, plus a reactive push away from the interceptor
    once it is within 6 m (offset up to 4 m, stronger the closer it gets).
- **Capture:** ground-truth range ≤ 1.0 m (both vehicles' odometry in one world
  frame), sampled at 10 Hz by `interception_metrics`. No vision in the loop:
  every pipeline method uses the `ground_truth` detector (see B10 in
  `ROADMAP.md`).
- **Outcomes:** `capture`; `timeout` (no capture in 120 s); `miss` (the
  interceptor came back to the ground without a capture: gave up and returned,
  sank, or crashed).
- **Timing:** liftoff-to-capture, from the moment the interceptor is 0.5 m above
  its start height, so arming latency does not count. Take-off does count, and
  it differs by method (see findings).
- **Randomisation:** each trial waits a seeded random delay of 0–15.7 s (one
  target lap) before the method starts, so the target's phase at engagement
  varies. Seed 1, same delays for every method.
- **Hygiene per trial:** the interceptor is back home and disarmed, the target
  is airborne in OFFBOARD, and PX4's preflight check passes; otherwise the sim
  is restarted. Each trial's launches are stopped (and any orphaned stage nodes
  killed) before the next one.

## Methods

| label | what flies the interceptor |
|---|---|
| `pipeline/rendezvous+gru+passthrough+px4_setpoint` | FSM head (`pipeline_fsm`: arm → AUTO.TAKEOFF to 4 m → OFFBOARD pursue/attack → RTL); KF → GRU forecast → rendezvous planner → PX4 position setpoints |
| `pipeline/rendezvous+const_vel+passthrough+px4_setpoint` | same, constant-velocity forecast (GRU ablation) |
| `pipeline/tail_chase+gru+passthrough+px4_setpoint` | same, tail-chase planner |
| `pipeline/rendezvous+gru+passthrough+se3` | same, geometric SE(3) controller (attitude + thrust) instead of PX4 position control; thrust scaled by PX4's hover thrust (0.79), acceleration limit 12 m/s² applied vertical-first |
| `bt/rendezvous+gru` | Behaviour-Tree head (`d2dtracker_states` `bt_node`, climbs in OFFBOARD) on the rendezvous + GRU reference |
| `pn` | `d2dtracker_interception` guidance, June PN law (N = 3, fixed 6 m/s closing, correction added to the measured velocity) |
| `pursuit` | same node, pure pursuit at 6 m/s |
| `pn_int` | same node, textbook constant-speed PN (N = 3) integrated on the commanded velocity at 6 m/s (`law:=pn_int`) |
| `rl/model_evasive_40k` | `d2dtracker_rl` `policy_head`: the evasive-stage SAC policy (trained in AeroGym against a 0.5 m/s target), velocity commands up to 8 m/s |

## Results

![capture rate](benchmarks/2026-10-30_pipeline_vs_pn/report/capture_rate.png)

| scenario | method | captures | capture rate (95 % Wilson CI) | liftoff-to-capture median [IQR] (s) | closest approach median / max (m) | non-captures |
|---|---|---|---|---|---|---|
| circle | rendezvous+gru | 10/10 | 100 % (72–100) | 9.6 [9.0–10.5] | 0.96 / 1.00 | |
| circle | rendezvous+const_vel | 10/10 | 100 % (72–100) | 9.5 [9.3–10.4] | 0.95 / 0.99 | |
| circle | tail_chase+gru | 10/10 | 100 % (72–100) | 12.6 [10.3–18.7] | 0.99 / 1.00 | |
| circle | rendezvous+gru+se3 | 9/10 | 90 % (60–98) | 9.1 [8.6–9.7] | 0.83 / 1.00 | 1 miss |
| circle | **bt**/rendezvous+gru | 10/10 | 100 % (72–100) | **4.6** [4.3–5.5] | 0.97 / 1.00 | |
| circle | pn | 4/10 | 40 % (17–69) | 13.4 [11.1–16.8] | 1.61 / 2.14 | 6 timeout |
| circle | pursuit | 10/10 | 100 % (72–100) | 11.8 [9.0–13.7] | 0.82 / 0.99 | |
| circle | pn_int | 9/10 | 90 % (60–98) | 7.7 [7.1–7.8] | 0.87 / 1.06 | 1 miss |
| circle | rl/model_evasive_40k | 2/10 | 20 % (6–51) | 28.9 [27.6–30.1] | 3.86 / 18.48 | 4 miss, 4 timeout |
| evade | rendezvous+gru | 8/10 | 80 % (49–94) | 22.1 [10.7–45.2] | 0.99 / 1.06 | 2 miss |
| evade | rendezvous+const_vel | 9/10 | 90 % (60–98) | 10.3 [10.0–13.3] | 0.97 / 1.20 | 1 miss |
| evade | tail_chase+gru | 8/10 | 80 % (49–94) | 28.5 [24.3–40.3] | 0.99 / 1.12 | 2 miss |
| evade | rendezvous+gru+se3 | 10/10 | 100 % (72–100) | 9.3 [8.4–16.9] | 0.78 / 1.00 | |
| evade | **bt**/rendezvous+gru | 10/10 | 100 % (72–100) | 9.9 [4.5–12.9] | 0.96 / 1.00 | |
| evade | pn | 1/10 | 10 % (2–40) | 13.5 | 2.59 / 3.57 | 9 timeout |
| evade | pursuit | 10/10 | 100 % (72–100) | 9.6 [7.7–15.8] | 0.88 / 0.99 | |
| evade | pn_int | 8/10 | 80 % (49–94) | **7.7** [6.8–8.3] | 0.84 / 1.83 | 2 miss |
| evade | rl/model_evasive_40k | 0/10 | 0 % (0–28) | — | 2.39 / 7.46 | 6 miss, 4 timeout |

180 valid trials of 180. Five of them (tail_chase circle t08 and evade t08/t09,
pn circle t02, rendezvous+gru evade t07) never got a healthy scene in the four
October passes and ran on 1 Nov, after the clock-bridge fix (Caveats). The
machine-readable table is
[`report/summary.csv`](benchmarks/2026-10-30_pipeline_vs_pn/report/summary.csv),
and it also gives total time from launch.

![time from liftoff to capture](benchmarks/2026-10-30_pipeline_vs_pn/report/time_engaged.png)

Example trajectories (first capture per method; for a method with none, its
closest trial): [circle](benchmarks/2026-10-30_pipeline_vs_pn/report/xy_circle.png),
[evade](benchmarks/2026-10-30_pipeline_vs_pn/report/xy_evade.png). Range against
time: [circle](benchmarks/2026-10-30_pipeline_vs_pn/report/range_circle.png),
[evade](benchmarks/2026-10-30_pipeline_vs_pn/report/range_evade.png).

## Findings

- **The Behaviour-Tree head is the strongest method:** 20/20 captures, the
  fastest on the circle. Under evasion the BT head, pursuit and SE(3) are within
  0.6 s of each other (9.3–9.9 s) and `pn_int` is quicker (7.7 s) but captures
  8/10. Most of its lead over the FSM head (4.6 s vs 9.6 s on the circle) is
  take-off. The altitude traces show the FSM head spends 3.7–5.3 s in PX4's
  AUTO.TAKEOFF before OFFBOARD pursuit starts, while the BT climbs on its own
  OFFBOARD setpoints and often captures before it reaches 3.5 m. After take-off
  both heads fly the same rendezvous + GRU reference.
- **Pure pursuit at 6 m/s also captures every time (20/20)**, a few seconds
  slower than the BT head. It is a strong, simple baseline at this target speed.
- **Textbook PN (`pn_int`) is the quickest once airborne:** 17/20 in a median
  7.7 s from liftoff in both scenarios, the fastest under evasion (BT head
  9.9 s, pursuit 9.6 s) and with the tightest spread. Its 3 misses passed just
  outside the capture sphere (1.02–1.83 m) and came back down. In AeroGym the
  same law overshoots out of the 60 m arena against faster, straight-fleeing
  targets (`d2dtracker_rl/RESULTS.md`, C4); here the target circles and the
  scene has no arena limit.
- **The June PN law fails** (5/20). With correct velocities it settles into an
  orbit around the circling target: closest approach is 1.6 m on the circle and
  2.6 m under evasion. Its June capture came from the odometry-frame bug (B3/B4
  in `ROADMAP.md`).
- **GRU vs constant-velocity forecast:** equal on the circle (10/10 each,
  9.6 vs 9.5 s). Under evasion, constant velocity is better: 9/10 in a median
  10.3 s, against 8/10 in 22.1 s with a wide spread. One untested explanation is
  that the GRU's 1 s forecast lags the evasive push, where constant velocity
  follows each change at once. Logging forecast error per trial would settle it.
  The training data of the shipped GRU weights is not documented here.
- **Rendezvous is faster than tail chase:** 8/10 each under evasion, in a
  median 22.1 s against 28.5 s; on the circle 10/10 each, 9.6 s against 12.6 s.
- **The SE(3) geometric controller holds up once its thrust model is right:**
  19/20, in a median 9.1 s (circle) and 9.3 s (evade) from liftoff, level
  with the FSM head's PX4 position control on the circle and faster under
  evasion. Its first campaign (6/10, 1/10) assumed 34 N of thrust; PX4's
  hover-thrust estimate for `x500_d435` (0.79) gives 30.5 N, a thrust/weight
  of 1.26. Chasing at up to 45° of tilt then saturated the throttle, and the
  acceleration clip scaled gravity's share down with the rest, so the vehicle
  sank: its misses passed 2–4 m *under* the target, often within 1 m
  horizontally. `hover_thrust` and a vertical-first acceleration clip
  (`interception_control` `cf5d9d4`) fixed it. It also needs the correct mass
  (2.466 kg) to fly at all (B6).
- **The RL policy does not transfer to this scene** (2/20). It was trained
  against a 0.5 m/s target; here the target flies at 2.4 m/s. The
  speed-generalisation sweep and the speed curriculum (C4, C6) target this gap.

## Caveats

- Simulation only, with ground-truth target detection.
- One target speed (2.4 m/s) and one geometry.
- Capture is a 1.0 m sphere at 10 Hz. `closest approach` at capture only shows
  where sampling crossed the radius.
- 138 attempts were excluded and re-run: 112 scenes that could not be made
  healthy, 23 SE(3) rows superseded (3 by the mass fix, the 20 of the first
  campaign by the thrust fix; their trace files were overwritten by the
  re-flights and stay in git history), and 3 runner errors. `summary.csv` counts them per method (`errors`).
  The main causes:
  - After any capture with contact the interceptor's PX4 failed its preflight
    check (EKF magnetometer fault), forcing a sim restart.
  - About 40 % of restarts never got the target airborne: it stayed disarmed
    in AUTO.LOITER. Found on 1 Nov: the ROS `/clock` bridge, started with the
    launch before PX4 brought the gz server up, sometimes passed no clock on.
    Every `use_sim_time` node froze, including the target node, which then
    never streamed setpoints or asked for OFFBOARD (6 of 14 starts that
    morning). The bridge now waits until gz publishes `/clock`
    (`interceptor.launch.py`); the next 20 starts were all healthy.
- Excluding faults keeps each valid trial clean, but a method is never run on a
  degraded scene, so these are best-case rates.
- The campaign shared the machine with up to four other simulation sessions
  (load average 8–60), which was not controlled for. Gazebo and PX4 SITL run in
  lockstep; the ROS controllers run on wall-clock timers.

## Reproduce

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp RRV_SHIM_BYPASS=1
source ros2_ws/install/setup.bash
python3 ros2_ws/src/drone_interception_sim/scripts/run_benchmark.py \
    ros2_ws/src/drone_interception_sim/benchmarks/campaigns/pipeline_vs_pn.yaml \
    --out ~/benchmarks_runs/pipeline_vs_pn      # resumable: re-run to fill gaps
python3 ros2_ws/src/drone_interception_sim/scripts/analyze_benchmark.py ~/benchmarks_runs/pipeline_vs_pn
```

The runner starts and stops the sim itself, using `partition` and `domain` from
the campaign file. This campaign needed four passes between 30 Oct 05:00 and
31 Oct 05:00 (with pauses for runner fixes), mostly spent on sim restarts; the
`pn_int` rows and the last 5 cells took about an hour on 1 Nov.

## June 2026 single runs (history)

The first matrix had one or two runs per method on the circle, with the FSM
head. Rendezvous + GRU captured twice in 15.6 and 16.1 s, tail chase + GRU in
15.3 s, rendezvous + const_vel in 12.75 s, and PN in 10.7 s. `pn_lead` as a
position planner timed out at 3.9 m. Those times count from launch, not from
liftoff, and the PN capture depended on the frame bug. Raw rows:
[`benchmarks/interception_results_2026-06-11.csv`](benchmarks/interception_results_2026-06-11.csv).
