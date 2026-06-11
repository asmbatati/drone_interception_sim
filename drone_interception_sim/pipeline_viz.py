#!/usr/bin/env python3
"""RViz visualization for the interception pipeline.

Republishes everything worth watching as a single colour-coded MarkerArray in
the common world frame (``map``), so one RViz MarkerArray display shows it all:

  * target ACTUAL pose      green sphere   (target odom + target_spawn)
  * target ESTIMATE         yellow sphere  (KF good_tracks, already world frame)
  * interceptor pose        blue sphere    (interceptor odom + interceptor_spawn)
  * predicted trajectory    cyan line      (prediction stage path)
  * planned path            magenta line   (planning reference path)
  * intercept point         red sphere     (planner rendezvous)

World positions are computed from odom + spawn offsets (the same convention the
ground_truth detector uses), so this needs no TF wiring. The gap between the
green (actual) and yellow (estimate) spheres is the live estimation error.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point, PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _c(r, g, b, a=0.9):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


class PipelineViz(Node):
    def __init__(self):
        super().__init__('pipeline_viz')
        self.frame = self.declare_parameter('world_frame', 'map').value
        self.interceptor_spawn = list(map(
            float, self.declare_parameter('interceptor_spawn', [0.0, 0.0, 0.0]).value))
        self.target_spawn = list(map(
            float, self.declare_parameter('target_spawn', [10.0, 0.0, 0.0]).value))

        self.target_odom = None
        self.interceptor_odom = None
        self.estimate = None       # (x,y,z) in world frame
        self.predicted = []        # list of (x,y,z)
        self.planned = []          # list of (x,y,z)
        self.intercept = None      # (x,y,z)

        sd = qos_profile_sensor_data
        self.create_subscription(Odometry, 'target_odom', self._tgt_cb, sd)
        self.create_subscription(Odometry, 'interceptor_odom', self._int_cb, sd)
        self.create_subscription(PoseArray, 'target_estimate', self._est_cb, sd)
        self.create_subscription(Path, 'predicted_path', self._pred_cb, sd)
        self.create_subscription(Path, 'planned_path', self._plan_cb, sd)
        self.create_subscription(PoseStamped, 'intercept_point', self._ip_cb, sd)

        self.pub = self.create_publisher(MarkerArray, 'markers', 1)
        self.create_timer(0.1, self._publish)
        self.get_logger().info(f"Pipeline viz up -> MarkerArray in '{self.frame}'.")

    # --- inputs (world = odom + spawn for the two craft; the rest are already map)
    def _tgt_cb(self, m):
        p = m.pose.pose.position
        self.target_odom = (p.x + self.target_spawn[0], p.y + self.target_spawn[1],
                            p.z + self.target_spawn[2])

    def _int_cb(self, m):
        p = m.pose.pose.position
        self.interceptor_odom = (p.x + self.interceptor_spawn[0],
                                 p.y + self.interceptor_spawn[1],
                                 p.z + self.interceptor_spawn[2])

    def _est_cb(self, m):
        if m.poses:
            p = m.poses[0].position
            self.estimate = (p.x, p.y, p.z)

    def _pred_cb(self, m):
        self.predicted = [(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z)
                          for ps in m.poses]

    def _plan_cb(self, m):
        self.planned = [(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z)
                        for ps in m.poses]

    def _ip_cb(self, m):
        p = m.pose.position
        self.intercept = (p.x, p.y, p.z)

    # --- marker builders ----------------------------------------------------
    def _sphere(self, ns, xyz, color, scale=0.6):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, xyz)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color = color
        return m

    def _label(self, ns, xyz, text, color):
        m = self._sphere(ns, (xyz[0], xyz[1], xyz[2] + 0.7), color)
        m.id = 1
        m.type = Marker.TEXT_VIEW_FACING
        m.scale.z = 0.4
        m.text = text
        return m

    def _line(self, ns, pts, color, width=0.08):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = width
        m.color = color
        m.points = [Point(x=float(a), y=float(b), z=float(c)) for a, b, c in pts]
        return m

    def _publish(self):
        arr = MarkerArray()
        if self.target_odom:
            g = _c(0.1, 0.9, 0.1)
            arr.markers += [self._sphere('target_actual', self.target_odom, g),
                            self._label('target_actual', self.target_odom, 'target (actual)', g)]
        if self.estimate:
            y = _c(1.0, 0.85, 0.0)
            arr.markers += [self._sphere('target_estimate', self.estimate, y, 0.5),
                            self._label('target_estimate', self.estimate, 'target (estimate)', y)]
            # Live estimation-error readout: the distance between the actual
            # target and the KF estimate, so estimator quality is visible at a
            # glance (no guessing from sphere overlap).
            if self.target_odom:
                err = math.dist(self.target_odom, self.estimate)
                e_color = _c(0.2, 1.0, 0.2) if err < 0.5 else _c(1.0, 0.3, 0.2)
                arr.markers.append(self._label(
                    'estimation_error',
                    (self.estimate[0], self.estimate[1], self.estimate[2] + 0.8),
                    f'est err {err:.2f} m', e_color))
        if self.interceptor_odom:
            b = _c(0.2, 0.4, 1.0)
            arr.markers += [self._sphere('interceptor', self.interceptor_odom, b),
                            self._label('interceptor', self.interceptor_odom, 'interceptor', b)]
        if self.intercept:
            r = _c(1.0, 0.1, 0.1)
            arr.markers += [self._sphere('intercept_point', self.intercept, r, 0.4),
                            self._label('intercept_point', self.intercept, 'intercept pt', r)]
        if len(self.predicted) >= 2:
            arr.markers.append(self._line('predicted_path', self.predicted, _c(0.0, 0.9, 0.9)))
        # Planned path = the interceptor's approach to the planner's reference.
        # The reference is often a single rendezvous point, so prepend the
        # interceptor position to always draw a visible approach segment.
        planned = ([self.interceptor_odom] if self.interceptor_odom else []) + self.planned
        if len(planned) >= 2:
            arr.markers.append(self._line('planned_path', planned, _c(1.0, 0.2, 1.0)))
        if arr.markers:
            self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = PipelineViz()
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
