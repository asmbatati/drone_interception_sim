# drone_interception_sim

Drone-to-drone **interception** simulation assets (interceptor + target multi-drone)
for ROS 2 **Jazzy** + Gazebo **Harmonic** + PX4 SITL. Built on top of
[`uav_gz_sim`](../uav_gz_sim) — it reuses that package's parametric `gz_sim` /
`mavros` launches, drone models (`x500_d435`, `x3_uav`) and PX4 airframes
(`4020`, `4021`) rather than duplicating them.

## What it does

Spawns **both** vehicles into a **single** Gazebo world, each as its own PX4 SITL
instance with its own MAVROS stack:

| | Interceptor | Target |
|---|---|---|
| namespace | `interceptor` | `target` |
| model | `x500_d435` | `x3_uav` |
| PX4 autostart | `4020` | `4022` |
| instance (`px4 -i`) | `0` | `1` |
| MAVROS `fcu_url` | `udp://:14541@127.0.0.1:14558` | `udp://:14542@127.0.0.1:14559` |
| `tgt_system` | `1` | `2` |
| spawn pose (ENU) | `0,0,0.1` | `10,0,0.1` |

The target flies an autonomous scripted trajectory (circle / figure-8) and
self-arms into OFFBOARD; the interceptor is left for the interception
controllers (`d2dtracker_interception`, `d2dtracker_states` BT/FSM,
`d2dtracker_rl`) to drive.

## PX4 directory

The launches default `PX4_DIR` to the project's **in-tree** PX4
(`<workspace>/PX4-Autopilot`, beside `ros2_ws`) regardless of any `PX4_DIR` in
your shell. Override per launch with `px4_dir:=/path/to/PX4-Autopilot`.

That in-tree PX4 must be built (`make px4_sitl`) and provide the airframes used
here: `4020_gz_x500_d435` (interceptor) and `4022_gz_x3_uav` (target), plus the
`x500_d435` and `x3_uav` gz models. (Note: the in-tree PX4 numbers `x3_uav` as
`4022`; a `uav_gz_sim`-provisioned PX4 uses `4021` — adjust `px4_autostart_id`
if you point at a differently-provisioned tree.)

## Build & run

```bash
cd ~/drone_interception_ws/ros2_ws
colcon build --packages-select drone_interception_sim
source install/setup.bash

# Rendered, with RViz (default)
ros2 launch drone_interception_sim interception.launch.py

# Headless, no RViz
ros2 launch drone_interception_sim interception.launch.py headless:=1 use_rviz:=false
```

You should see **one** Gazebo GUI with two drones. Verify both FCUs connect:

```bash
ros2 topic echo /interceptor/mavros/state --once
ros2 topic echo /target/mavros/state --once     # connected: true
```

## Gazebo partition isolation

The launches set **`GZ_PARTITION=d2d_intercept`** by default so this sim spins up
its **own** Gazebo server and never collides with another PX4+gz sim running on
the same machine (a different workspace, etc.). Without isolation, PX4 attaches
to whatever gz server is already up and the model spawn fails
(`/world/interception/create` not found → `Service call timed out`).

- Override or disable: `gz_partition:=my_part` (or `gz_partition:=` for the
  default shared partition).
- **External gz tools must match it**: `GZ_PARTITION=d2d_intercept gz topic -l`,
  `GZ_PARTITION=d2d_intercept gz sim -g` (GUI), etc.

## Scaling to N drones

The spawn scheme generalizes: drone `i` uses PX4 instance `i`, namespace of your
choice, `PX4_GZ_MODEL`/airframe for its model, MAVLink `fcu_url`
`udp://:1454{i}@127.0.0.1:1455{7+i}`, and `tgt_system = i+1`. To add more
vehicles, copy `target.launch.py`, bump `INSTANCE_ID`/`FCU_URL`/`TGT_SYSTEM`/
`MODEL`/`AUTOSTART_ID`, keep the **same** `gz_world` + `gz_partition` (so they
share one Gazebo server), and start each after the world owner with a
`TimerAction`. Only the first launch carries the world + RViz.

## Evasive target

The scripted target can reactively flee the interceptor. Enable it in
`config/target_trajectory.yaml` (`evade: true`, `evade_distance`, `evade_gain`)
or per-run; it then subscribes to the interceptor odom (remapped in
`target.launch.py`) and adds a repulsion offset when the interceptor closes in.
Useful as a harder RL/strategy benchmark than the fixed circle/figure-8.

## Visualizing the drones in RViz (no URDF)

The drones are spawned from PX4 SDF, so there is no ROS `robot_description` for
RViz. Instead, a `drone_markers` node publishes a quadcopter **MarkerArray**
attached to each `<ns>/base_link` frame (interceptor = blue, target = red); the
markers track the drone through TF. The RViz config shows `/interceptor/markers`
and `/target/markers`. To render the real mesh instead of the geometric quad,
pass `mesh_resource:=file:///abs/path/to/model.dae` to the node.

The propeller blades **spin from real flight data**: the node subscribes to
`mavros/state` (armed) and `mavros/vfr_hud` (throttle) and advances the rotor
angle at a rate proportional to the actual PX4 throttle (zero when disarmed),
tunable via `max_spin_rate`/`idle_spin_rate`. This is a uniform throttle-driven
spin; for true per-motor RPM, drive it from px4_msgs `ActuatorMotors`
(uXRCE-DDS) instead.

## Notes

- **Single Gazebo server**: both drones share the identical `world` arg; the
  interceptor starts first and owns the server, the target is delayed
  (`target_spawn_delay`, default 8 s) so it only spawns a model into the
  existing server. This avoids the two-GUI bug of the old `run_sim.launch.py`.
- The target trajectory is configured in [`config/target_trajectory.yaml`](config/target_trajectory.yaml).
