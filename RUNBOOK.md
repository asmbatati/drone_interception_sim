# Interception runbook

End-to-end recipes for the three interception methods + benchmarking. One
terminal per block. **Every terminal that talks to the sim must use the same
`RMW_IMPLEMENTATION` AND the same `ROS_DOMAIN_ID`** — otherwise nodes won't see
each other, and (critically) other simulators on the machine will collide on
`/clock`, causing "jump back in time" and PX4 refusing to arm.

Put this at the top of **every** terminal (the sim defaults to domain 77):

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp     # matches your .bashrc
export ROS_DOMAIN_ID=77                      # matches the sim's default
cd ~/drone_interception_ws/ros2_ws && source install/setup.bash
```

> Isolation: the sim runs on `GZ_PARTITION=d2d_intercept` (Gazebo transport) and
> `ROS_DOMAIN_ID=77` (ROS graph) so it coexists with other sims (drone_arm_ws,
> text2geometry, …). Override with `ros_domain_id:=N gz_partition:=name`, or
> `ros_domain_id:=''` to inherit the shell. External `gz` tools need
> `GZ_PARTITION=d2d_intercept`.

> Preconditions: PX4 built (`make px4_sitl` in the in-tree PX4), `uav_gz_sim`
> install.sh run once (models/airframes in PX4), workspace built. If you also
> run another PX4+gz sim (e.g. `drone_arm_ws`), this sim is isolated via
> `GZ_PARTITION=d2d_intercept` — external `gz` tools must set that to connect.

## 0. Bring up the scene (two drones, one Gazebo)
```bash
ros2 launch drone_interception_sim interception.launch.py
# health check (separate terminal):
src/drone_interception_sim/scripts/check_sim.sh      # expect connected: true x2
```
Single drone only (for RL high-level / quick checks):
```bash
ros2 launch drone_interception_sim interceptor.launch.py
```

## 1. Strategy A — geometric rendezvous (intercept point/path)
```bash
ros2 launch d2dtracker_interception interceptor.launch.py     # publishes intercept_point + path
```

## 2. Strategy B — Proportional Navigation (flies the interceptor)
```bash
ros2 launch d2dtracker_interception guidance.launch.py        # PN controller
```

## 3. Behaviour Tree mission (Search -> Pursue -> Attack -> Return)
```bash
ros2 launch d2dtracker_states behavior_tree.launch.py
```
(FSM variant: `ros2 launch d2dtracker_states state_machine.launch.py`)

## 4. Reinforcement Learning
```bash
# sim already up; train stage 1 (PPO, high-level)
ros2 launch d2dtracker_rl train_high_level.launch.py
tensorboard --logdir runs/
# full curriculum (static -> constant-velocity -> evasive -> low-level SAC):
ros2 run d2dtracker_rl curriculum --config \
  $(ros2 pkg prefix d2dtracker_rl)/share/d2dtracker_rl/config/curriculum.yaml
# evaluate a trained policy:
ros2 launch d2dtracker_rl evaluate.launch.py stage:=high \
  model:=checkpoints/high/final_model.zip
```

## 5. Benchmark any method (capture rate / time-to-intercept / miss distance)
Run alongside whichever controller is driving the interceptor:
```bash
ros2 launch drone_interception_sim metrics.launch.py method:=pn \
  csv_path:=/tmp/pn_run.csv
```
Repeat with `method:=bt`, `method:=rl`, `method:=rendezvous` to compare CSVs.

## 6. Optional: vision instead of ground-truth target
```bash
pip install ultralytics        # once
ros2 launch drone_interception_sim perception.launch.py device:=cpu \
  model:=<path>/smart_track/config/drone_detection_v3.pt
# then point the controller's target input at /interceptor/target_detection
```

## Stopping / relaunching

PX4 SITL holds a per-instance lock, and a crashed or Ctrl-C'd run can leave the
`px4` daemon alive — the next launch then dies with `PX4 server already running
for instance N` (the target/interceptor PX4 exits 255). Two safeguards:

- `interceptor.launch.py` **auto-clears** leftover px4/gz from a previous run of
  *this* sim (matched by `GZ_PARTITION`, so other projects are untouched) before
  starting — so just relaunching usually works.
- To stop a run cleanly (or clean up manually): `scripts/stop_sim.sh`.

If you run several PX4+gz sims at once, give each its own `gz_partition:=` /
`ros_domain_id:=` (and don't reuse PX4 instance numbers across them).

## Tips
- Two-drone cold start: if the target's MAVROS is slow to connect, raise
  `target_spawn_delay:=12`.
- Harder target: set `evade:=true` (see config/target_trajectory.yaml).
- Headless (no GUI): add `headless:=1 use_rviz:=false`.
- Run only ONE interceptor controller at a time (PN/BT/FSM/RL share the FCU).
