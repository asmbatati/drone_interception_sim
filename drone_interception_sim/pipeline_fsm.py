#!/usr/bin/env python3
"""FSM orchestration head for the interception pipeline.

This is the phase-sequencing "brain" that sits on top of the
detection->estimation->prediction->planning->MPC->control cascade. It does NOT
compute or stream setpoints itself (the control stage does that, continuously);
it only manages arming + PX4 flight mode + the mission phase, exactly the role
the old d2dtracker_states FSM played in the original framework:

    Idle -> Arm -> Takeoff(AUTO.TAKEOFF) -> Pursue(OFFBOARD) -> Attack(OFFBOARD)
         -> Return(AUTO.RTL) -> Done

Because PX4 ignores OFFBOARD setpoints unless it is in OFFBOARD mode, the cascade
can stream the whole time and this node just switches modes: it climbs with
AUTO.TAKEOFF, then engages OFFBOARD so the cascade flies the interceptor to the
planner's intercept reference, and switches to AUTO.RTL on capture. Transitions
are driven by altitude, target detection and interceptor->target range.

Selected via pipeline.launch.py orchestrator:=fsm. Alternative heads (bt, rl,
offboard) are selectable there too; see ARCHITECTURE.md.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class PipelineFSM(Node):
    def __init__(self):
        super().__init__('pipeline_fsm')
        self.takeoff_alt = float(self.declare_parameter('takeoff_alt', 4.0).value)
        self.attack_distance = float(self.declare_parameter('attack_distance', 3.0).value)
        self.capture_radius = float(self.declare_parameter('capture_radius', 1.0).value)
        self.min_setpoints = int(self.declare_parameter('min_setpoints', 50).value)
        self.target_timeout = float(self.declare_parameter('target_timeout', 1.0).value)

        self.phase = 'Idle'
        self.state = State()
        self.interceptor = None      # numpy-free: (x,y,z)
        self.target = None           # world-frame target position (x,y,z)
        self.last_target_t = None
        self.captured = False

        self.create_subscription(State, 'mavros/state', self._state_cb, 10)
        self.create_subscription(Odometry, 'interceptor_odom',
                                 self._iodom_cb, qos_profile_sensor_data)
        # Confirmed target estimate (world frame) from the estimation stage.
        self.create_subscription(PoseArray, 'target_estimate',
                                 self._target_cb, qos_profile_sensor_data)

        self.phase_pub = self.create_publisher(String, 'mission/phase', 10)
        self.arming = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode = self.create_client(SetMode, 'mavros/set_mode')

        self.create_timer(0.2, self._tick)
        self.get_logger().info('Pipeline FSM head up (phase=Idle).')

    # --- inputs -------------------------------------------------------------
    def _state_cb(self, m):
        self.state = m

    def _iodom_cb(self, m):
        p = m.pose.pose.position
        self.interceptor = (p.x, p.y, p.z)

    def _target_cb(self, m):
        if m.poses:
            p = m.poses[0].position
            self.target = (p.x, p.y, p.z)
            self.last_target_t = self.get_clock().now()

    # --- helpers ------------------------------------------------------------
    def _detected(self):
        if self.target is None or self.last_target_t is None:
            return False
        age = (self.get_clock().now() - self.last_target_t).nanoseconds * 1e-9
        return age < self.target_timeout

    def _range(self):
        if self.interceptor is None or self.target is None:
            return math.inf
        return math.dist(self.interceptor, self.target)

    def _altitude(self):
        return self.interceptor[2] if self.interceptor else 0.0

    def _request_mode(self, mode):
        if self.state.mode != mode and self.set_mode.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = mode
            self.set_mode.call_async(req)

    def _request_arm(self):
        if not self.state.armed and self.arming.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            self.arming.call_async(req)

    def _to(self, phase):
        if phase != self.phase:
            self.get_logger().info(f'phase: {self.phase} -> {phase}')
            self.phase = phase

    # --- the mission FSM ----------------------------------------------------
    def _tick(self):
        self.phase_pub.publish(String(data=self.phase))
        if not self.state.connected or self.interceptor is None:
            return

        if self.phase == 'Idle':
            self._to('Arm')

        elif self.phase == 'Arm':
            # PX4 won't arm while finishing an auto land/return; leave it first.
            if self.state.mode in ('AUTO.RTL', 'AUTO.LAND'):
                self._request_mode('AUTO.LOITER')
            else:
                self._request_arm()
            if self.state.armed:
                self._to('Takeoff')

        elif self.phase == 'Takeoff':
            self._request_mode('AUTO.TAKEOFF')
            if self._altitude() >= 0.9 * self.takeoff_alt:
                self._to('Pursue')

        elif self.phase == 'Pursue':
            # Hand the craft to the cascade: OFFBOARD makes PX4 act on the
            # control stage's setpoints (which are already streaming).
            self._request_mode('OFFBOARD')
            if not self._detected():
                return  # keep OFFBOARD-loitering on the last command; wait for target
            if self._range() <= self.attack_distance:
                self._to('Attack')

        elif self.phase == 'Attack':
            self._request_mode('OFFBOARD')
            if self._range() <= self.capture_radius:
                self.captured = True
                self.get_logger().info(f'CAPTURE (range={self._range():.2f} m) -> returning')
                self._to('Return')
            elif self._range() > 2.0 * self.attack_distance:
                self._to('Pursue')   # lost the close approach, fall back to pursue

        elif self.phase == 'Return':
            self._request_mode('AUTO.RTL')
            if not self.state.armed:
                self._to('Done')


def main(args=None):
    rclpy.init(args=args)
    node = PipelineFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
