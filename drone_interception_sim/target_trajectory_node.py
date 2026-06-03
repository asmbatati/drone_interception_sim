#!/usr/bin/env python3
"""Scripted target drone that flies a closed trajectory (circle / figure-8).

Re-homed and extended from the retired d2dtracker_sim/offboard_control_node.py.
Unlike the original (which only streamed setpoints and relied on an external
arming actor), this node is self-contained: it streams OFFBOARD setpoints, then
auto-switches the FCU to OFFBOARD and arms it, so the target flies on its own.

All topics/services are relative, so the node works under any namespace
(e.g. launched under `target` -> /target/mavros/...).
"""
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)

from .trajectories import Circle3D, Infinity3D


class TargetTrajectory(Node):

    def __init__(self):
        super().__init__('target_trajectory_node')

        self.declare_parameter('radius', 5.0)
        self.declare_parameter('omega', 0.5)
        self.declare_parameter('trajectory_type', 'circle')  # 'circle' | 'infty'
        self.declare_parameter('normal_vector', [0., 0., 1.])
        self.declare_parameter('center', [0., 0., 3.0])
        self.declare_parameter('auto_arm', True)
        # Reactive evasion: push away from the interceptor when it gets close.
        self.declare_parameter('evade', False)
        self.declare_parameter('evade_distance', 6.0)
        self.declare_parameter('evade_gain', 4.0)
        self.declare_parameter('target_spawn', [10.0, 0.0, 0.0])
        self.declare_parameter('interceptor_spawn', [0.0, 0.0, 0.0])

        self.radius_ = self.get_parameter('radius').value
        self.omega_ = self.get_parameter('omega').value
        self.trajectory_type_ = self.get_parameter('trajectory_type').value
        self.normal_vector_ = self.get_parameter('normal_vector').value
        self.center_ = self.get_parameter('center').value
        self.auto_arm_ = self.get_parameter('auto_arm').value
        self.evade_ = self.get_parameter('evade').value
        self.evade_distance_ = self.get_parameter('evade_distance').value
        self.evade_gain_ = self.get_parameter('evade_gain').value
        self.target_spawn_ = list(self.get_parameter('target_spawn').value)
        self.interceptor_spawn_ = list(self.get_parameter('interceptor_spawn').value)

        if self.trajectory_type_ == 'circle':
            self.trajectory_generator_ = Circle3D(
                np.array(self.normal_vector_), np.array(self.center_),
                radius=self.radius_, omega=self.omega_)
        elif self.trajectory_type_ == 'infty':
            self.trajectory_generator_ = Infinity3D(
                np.array(self.normal_vector_), np.array(self.center_),
                radius=self.radius_, omega=self.omega_)
        else:
            raise ValueError("trajectory_type must be 'circle' or 'infty'")

        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.odom_ = Odometry()
        self.state_ = State()
        self.interceptor_odom_ = None

        self.create_subscription(State, 'mavros/state',
                                 self.stateCallback, qos_state)
        self.create_subscription(Odometry, 'mavros/local_position/odom',
                                 self.odomCallback, qos_profile_sensor_data)
        if self.evade_:
            self.create_subscription(Odometry, 'interceptor_odom',
                                     self.interceptorCallback, qos_profile_sensor_data)

        self.setpoint_pub_ = self.create_publisher(
            PositionTarget, 'mavros/setpoint_raw/local', qos_profile_sensor_data)
        self.path_pub_ = self.create_publisher(Path, 'target_path', 10)

        self.arming_client_ = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client_ = self.create_client(SetMode, 'mavros/set_mode')

        self.t0_ = Clock().now().nanoseconds / 1e9
        self.counter_ = 0
        self.path_msg_ = Path()

        self.create_timer(0.02, self.cmdloopCallback)          # 50 Hz setpoint stream
        self.create_timer(1.0, self.armOffboardCallback)        # 1 Hz arm/mode manager

    def stateCallback(self, msg: State):
        self.state_ = msg

    def odomCallback(self, msg: Odometry):
        self.odom_ = msg

    def interceptorCallback(self, msg: Odometry):
        self.interceptor_odom_ = msg

    def _evasion_offset(self):
        """Horizontal displacement (target-local) pushing away from the interceptor."""
        if not self.evade_ or self.interceptor_odom_ is None:
            return np.zeros(3)
        ip = self.interceptor_odom_.pose.pose.position
        tp = self.odom_.pose.pose.position
        # Convert both to a common (world) frame via spawn offsets.
        i_world = np.array([ip.x + self.interceptor_spawn_[0],
                            ip.y + self.interceptor_spawn_[1],
                            ip.z + self.interceptor_spawn_[2]])
        t_world = np.array([tp.x + self.target_spawn_[0],
                            tp.y + self.target_spawn_[1],
                            tp.z + self.target_spawn_[2]])
        away = t_world - i_world
        dist = float(np.linalg.norm(away))
        if dist >= self.evade_distance_ or dist < 1e-6:
            return np.zeros(3)
        # Stronger push the closer the interceptor is (translation = frame-invariant).
        scale = self.evade_gain_ * (1.0 - dist / self.evade_distance_)
        return (away / dist) * scale

    def armOffboardCallback(self):
        """Switch to OFFBOARD and arm once enough setpoints have been streamed."""
        if not self.auto_arm_:
            return
        # PX4 requires a setpoint stream before OFFBOARD is accepted.
        if self.counter_ < 100:
            return
        if self.state_.mode != 'OFFBOARD' and self.set_mode_client_.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            self.set_mode_client_.call_async(req)
            self.get_logger().info('Target: requesting OFFBOARD mode')
            return
        if not self.state_.armed and self.arming_client_.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            self.arming_client_.call_async(req)
            self.get_logger().info('Target: requesting arm')

    def cmdloopCallback(self):
        t = Clock().now().nanoseconds / 1e9 - self.t0_
        point = np.asarray(self.trajectory_generator_.generate_trajectory_setpoint(t),
                           dtype=float)
        point = point + self._evasion_offset()   # zero unless evading

        frame_id = self.odom_.header.frame_id or 'map'
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = frame_id
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY +
                        PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_VX +
                        PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                        PositionTarget.IGNORE_YAW_RATE)
        sp.position.x = float(point[0])
        sp.position.y = float(point[1])
        sp.position.z = float(point[2])
        sp.yaw = float(np.arctan2(point[1] - self.odom_.pose.pose.position.y,
                                  point[0] - self.odom_.pose.pose.position.x))
        self.setpoint_pub_.publish(sp)
        self.counter_ += 1

        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = sp.header.stamp
        ps.pose.position = sp.position
        self.path_msg_.header = ps.header
        self.path_msg_.poses.append(ps)
        if len(self.path_msg_.poses) > 500:
            self.path_msg_.poses.pop(0)
        self.path_pub_.publish(self.path_msg_)


def main(args=None):
    rclpy.init(args=args)
    node = TargetTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
