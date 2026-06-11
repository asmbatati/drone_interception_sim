#!/usr/bin/env python3
"""Offboard/arm manager for the interception pipeline.

The control stage streams setpoints to mavros/setpoint_raw/local but does not
arm or change mode (that is the mission controller's job). For a self-contained
pipeline run this lightweight node watches that setpoint stream and, once PX4
has seen enough setpoints (it requires a stream before OFFBOARD is accepted) and
the FCU is connected, switches to OFFBOARD and arms — so the interceptor flies
the pipeline's setpoints. It never publishes setpoints itself (no conflict with
the control stage).
"""
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode


class OffboardManager(Node):
    def __init__(self):
        super().__init__('offboard_manager')
        self.min_setpoints = int(self.declare_parameter('min_setpoints', 100).value)

        self.state_ = State()
        self.sp_count_ = 0

        self.create_subscription(State, 'mavros/state', self._state_cb, 10)
        # Match the control stage's reliable publisher (depth 10) so the
        # setpoint count is reliable under FastRTPS.
        self.create_subscription(PositionTarget, 'mavros/setpoint_raw/local',
                                 self._sp_cb, 10)

        self.arming_client_ = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client_ = self.create_client(SetMode, 'mavros/set_mode')

        self.create_timer(0.5, self._tick)
        self.get_logger().info('Offboard manager up: waiting for the control setpoint stream...')

    def _state_cb(self, msg):
        self.state_ = msg

    def _sp_cb(self, _msg):
        self.sp_count_ += 1

    def _tick(self):
        if not self.state_.connected:
            return
        # PX4 needs a setpoint stream before OFFBOARD is accepted.
        if self.sp_count_ < self.min_setpoints:
            return
        if self.state_.mode != 'OFFBOARD' and self.set_mode_client_.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            self.set_mode_client_.call_async(req)
            self.get_logger().info('Interceptor: requesting OFFBOARD mode')
            return
        if not self.state_.armed and self.arming_client_.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            fut = self.arming_client_.call_async(req)
            fut.add_done_callback(self._arm_cb)
            self.get_logger().info('Interceptor: requesting arm')

    def _arm_cb(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info('Interceptor armed; flying the pipeline.')
            else:
                self.get_logger().warn(
                    f'Interceptor arm rejected (result={resp.result}).')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'arm service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = OffboardManager()
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
