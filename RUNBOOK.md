# Interception runbook

End-to-end recipes for the three interception methods + benchmarking. One
terminal per block; **every terminal must use the same `RMW_IMPLEMENTATION`**
(your `.bashrc` now exports `rmw_zenoh_cpp`). Each launch sources are assumed:
`cd ~/drone_interception_ws/ros2_ws && source install/setup.bash`.

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

## Tips
- Two-drone cold start: if the target's MAVROS is slow to connect, raise
  `target_spawn_delay:=12`.
- Harder target: set `evade:=true` (see config/target_trajectory.yaml).
- Headless (no GUI): add `headless:=1 use_rviz:=false`.
- Run only ONE interceptor controller at a time (PN/BT/FSM/RL share the FCU).
