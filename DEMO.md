# Demo — run the full interception pipeline to a capture

Step-by-step, copy-pasteable. This reproduces the validated run:
`Idle → Arm → Takeoff → Pursue → Attack → CAPTURE → Return`. Two terminals.

> **Two rules that bite if ignored**
> 1. **Every terminal** that talks to the sim must export the **same RMW + domain**:
>    `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=77`
>    (zenoh drops `/clock`+odom on this machine; the sim wrapper forces FastRTPS).
> 2. Launch the stages **only** via `pipeline.launch.py` (one entry point).
>    Hand-launching stages across runs leaves orphan nodes that corrupt the estimate.

---

## 0. Build (once)

```bash
cd ~/drone_interception_ws/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build            # builds osqp-eigen, interception_*, drone_interception_sim, …
source install/setup.bash
```

---

## 1. Terminal A — start the simulator

The wrapper exports FastRTPS before `ros2 launch` (required — see rule 1):

```bash
cd ~/drone_interception_ws/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=77
src/drone_interception_sim/scripts/run_sim.sh gpu:=false        # headless-GPU; drop gpu:= for GUI
```

Wait ~30 s for both PX4s to boot. Verify in a scratch terminal (same RMW+domain):

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=77
source /opt/ros/jazzy/setup.bash && source ~/drone_interception_ws/ros2_ws/install/setup.bash
ros2 topic hz /interceptor/mavros/local_position/odom    # ~30 Hz when up
ros2 topic hz /target/mavros/local_position/odom         # ~30 Hz; the target auto-arms + flies
```

---

## 2. Terminal B — run the pipeline (FSM head → capture)

```bash
cd ~/drone_interception_ws/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=77

ros2 launch drone_interception_sim pipeline.launch.py orchestrator:=fsm metrics:=true \
    csv_path:=/tmp/capture.csv
```

You'll see each stage report "stage up", then the FSM head drive the mission. Its
phase prints to the console and to a topic:

```bash
# in another terminal (same RMW+domain):
ros2 topic echo /interceptor/mission/phase
#   Idle -> Arm -> Takeoff -> Pursue -> Attack -> Return -> Done
```

**Expected:** the interceptor arms, `AUTO.TAKEOFF` climbs to ~4–6 m, switches to
`OFFBOARD`, the cascade flies it to the target (range ~12 m → < 1 m), the FSM logs
`CAPTURE (range=0.xx m) -> returning`, then `AUTO.RTL` home.

---

## 3. Watch the capture (range over time)

Save and run this monitor (prints phase + true interceptor→target range):

```bash
cat > /tmp/watch.py <<'PY'
import rclpy, time, math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from mavros_msgs.msg import State
class V(Node):
    def __init__(self):
        super().__init__('watch'); self.io=self.to=None; self.ph='?'; self.mode='?'; self.armed=False
        self.create_subscription(Odometry,'/interceptor/mavros/local_position/odom',lambda m:setattr(self,'io',m),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/target/mavros/local_position/odom',lambda m:setattr(self,'to',m),qos_profile_sensor_data)
        self.create_subscription(String,'/interceptor/mission/phase',lambda m:setattr(self,'ph',m.data),10)
        self.create_subscription(State,'/interceptor/mavros/state',lambda m:(setattr(self,'mode',m.mode),setattr(self,'armed',m.armed)),10)
rclpy.init(); n=V(); SP=10.0   # target spawn x-offset
for _ in range(40):
    t=time.time()
    while time.time()-t<2: rclpy.spin_once(n,timeout_sec=0.1)
    if n.io and n.to:
        ip=n.io.pose.pose.position; tp=n.to.pose.pose.position
        d=math.dist((ip.x,ip.y,ip.z),(tp.x+SP,tp.y,tp.z))
        print(f"phase={n.ph:8s} mode={n.mode:12s} armed={n.armed} alt={ip.z:4.1f} range={d:5.1f} m")
rclpy.shutdown()
PY
python3 /tmp/watch.py
```

Sample of a successful run:
```
phase=Pursue   mode=OFFBOARD     armed=True alt= 6.0 range= 12.0 m
phase=Pursue   mode=OFFBOARD     armed=True alt= 2.3 range=  2.4 m
phase=Return   mode=AUTO.RTL     armed=True alt= 2.1 range=  2.3 m   # captured, returning
```

---

## 3b. See it in RViz (actual/estimate/interceptor/paths/intercept)

The markers display in **the sim's RViz** (launched with the sim) — no separate
RViz needed. A `pipeline_viz` node (on by default, `viz:=true`) republishes
everything as one colour-coded `MarkerArray` in the `map` frame on
`/interceptor/interception/markers`, and the sim's `interception.rviz` config
includes the display:

| colour | what |
|---|---|
| **green** sphere | target **actual** pose (target odom + spawn) |
| **yellow** sphere | target **estimate** (detection → KF) — gap to green = live estimation error |
| **blue** sphere | **interceptor** pose |
| **red** sphere | planned **intercept point** |
| **cyan** line | **predicted** target trajectory (deliberately ahead of the target) |
| **magenta** line | **planned** path (interceptor → reference) |
| `est err X m` text | live ‖actual − estimate‖ readout (green when < 0.5 m) |

> Judge the **estimator** by the `est err` text / green-yellow overlap only. The
> cyan predicted path and red intercept point lead the target **by design**.

```bash
# the sim's RViz shows the markers as soon as the pipeline runs:
ros2 launch drone_interception_sim pipeline.launch.py orchestrator:=fsm
# quick headless check that the markers are flowing (7 namespaces):
ros2 topic echo /interceptor/interception/markers --once | grep ns:
```
RViz Fixed Frame must be `map` (the sim's config sets it). If the sim's RViz was
opened before this display existed, restart the sim once. See `MARKERS.md` for
the full marker reference.

---

## 4. Inspect any stage (the pluggable cascade)

```bash
ros2 topic echo /interceptor/detection_node/detections_poses --once   # detection
ros2 topic echo /interceptor/kf/good_tracks --once                    # confirmed estimate (n, pos, vel)
ros2 topic echo /interceptor/predicted_trajectory --once              # forecast
ros2 topic echo /interceptor/intercept_point --once                   # planner rendezvous
ros2 topic echo /interceptor/command_trajectory --once                # MPC output
ros2 topic hz   /interceptor/mavros/setpoint_raw/local                # control -> PX4
```

---

## 5. Swap a backend or the orchestrator (no other change)

```bash
# different algorithm in one stage:
ros2 launch drone_interception_sim pipeline.launch.py predictor:=poly
ros2 launch drone_interception_sim pipeline.launch.py planner:=tail_chase controller:=se3
ros2 launch drone_interception_sim pipeline.launch.py mpc:=mpc_12state          # real OSQP MPC

# different orchestration head:
ros2 launch drone_interception_sim pipeline.launch.py orchestrator:=bt          # BT direct-drive (no MPC/control)
ros2 launch drone_interception_sim pipeline.launch.py orchestrator:=offboard    # arm+OFFBOARD only, cascade flies
ros2 launch drone_interception_sim pipeline.launch.py orchestrator:=none        # cascade only, no arming

# choices:
#   orchestrator: fsm | offboard | bt | none | (rl: reserved)
#   detector:  ground_truth | depth | yolo      estimator: const_vel | const_accel
#   predictor: gru (default; trained, old-impl parity) | const_vel | const_accel | poly
#   planner:   rendezvous | tail_chase | pn_lead | head_on
#   mpc:       passthrough | mpc_12state         controller: px4_setpoint | pn_velocity | se3
```

---

## 6. Shut down

```bash
# Ctrl-C terminal B (pipeline), then stop the sim from the workspace:
src/drone_interception_sim/scripts/stop_sim.sh
# verify clean:
ros2 node list            # (with the same RMW+domain) -> empty
```

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `ros2 topic`/`node list` shows nothing | RMW/domain mismatch — `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=77` in **this** terminal |
| FSM never leaves `Idle`/never arms | a previous run left the craft in `AUTO.RTL` or orphan nodes are running — `stop_sim.sh` (and only launch via `pipeline.launch.py`) |
| Estimate/intercept jumps wildly | duplicate stage nodes from hand-launching — kill them; use the single `pipeline.launch.py` |
| Interceptor arms but holds position | it must take off first — use `orchestrator:=fsm` (does `AUTO.TAKEOFF` before OFFBOARD), not `orchestrator:=offboard` from the ground |
| `/clock` flaky / sim-time nodes stall | pipeline defaults to wall clock (`use_sim_time:=false`); the sim must run under FastRTPS (`run_sim.sh`) |

See `ARCHITECTURE.md` for the full design and `RUNBOOK.md` for the original
per-method (strategy/BT/RL) recipes.
