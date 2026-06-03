#!/usr/bin/env python3
"""Method-agnostic interception metrics / benchmark logger.

Works regardless of which controller drives the interceptor (PN strategy, FSM,
Behaviour Tree, or RL): it just watches both drones' odometry and records the
engagement. Use it to compare methods on the same scenario.

Reports (printed on capture/timeout and written to CSV):
  * outcome            capture | timeout
  * time_to_intercept  seconds from first valid range to capture
  * min_range          closest approach (miss distance if no capture)

CSV columns: t, range, ix, iy, iz, tx, ty, tz

Launch under the interceptor namespace (or remap the two odom topics). Both
odometries are in their own EKF/spawn frame, so spawn offsets are added to get a
common frame (defaults match drone_interception_sim).
"""
import csv
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class InterceptionMetrics(Node):
    """Log range / capture / time-to-intercept for one engagement."""

    def __init__(self):
        super().__init__('interception_metrics')
        self.declare_parameter('method', 'unknown')
        self.declare_parameter('capture_radius', 1.0)
        self.declare_parameter('timeout', 120.0)
        self.declare_parameter('csv_path', '/tmp/interception_metrics.csv')
        self.declare_parameter('interceptor_spawn', [0.0, 0.0, 0.0])
        self.declare_parameter('target_spawn', [10.0, 0.0, 0.0])

        self.method = self.get_parameter('method').value
        self.capture_radius = self.get_parameter('capture_radius').value
        self.timeout = self.get_parameter('timeout').value
        self.csv_path = self.get_parameter('csv_path').value
        self.ispawn = list(self.get_parameter('interceptor_spawn').value)
        self.tspawn = list(self.get_parameter('target_spawn').value)

        self.interceptor = None
        self.target = None
        self.t0 = None
        self.min_range = float('inf')
        self.captured = False
        self.done = False

        self.create_subscription(Odometry, 'interceptor_odom',
                                 self._int_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'target_odom',
                                 self._tgt_cb, qos_profile_sensor_data)

        self._csv = open(self.csv_path, 'w', newline='')
        self._writer = csv.writer(self._csv)
        self._writer.writerow(['t', 'range', 'ix', 'iy', 'iz', 'tx', 'ty', 'tz'])

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"Metrics started (method={self.method}, capture_radius="
            f"{self.capture_radius} m) -> {self.csv_path}")

    def _int_cb(self, msg):
        self.interceptor = msg

    def _tgt_cb(self, msg):
        self.target = msg

    def _world(self, odom, spawn):
        p = odom.pose.pose.position
        return (p.x + spawn[0], p.y + spawn[1], p.z + spawn[2])

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        if self.done or self.interceptor is None or self.target is None:
            return
        ix, iy, iz = self._world(self.interceptor, self.ispawn)
        tx, ty, tz = self._world(self.target, self.tspawn)
        rng = math.sqrt((tx - ix) ** 2 + (ty - iy) ** 2 + (tz - iz) ** 2)

        now = self._now()
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        self.min_range = min(self.min_range, rng)
        self._writer.writerow([f'{t:.3f}', f'{rng:.4f}',
                               f'{ix:.3f}', f'{iy:.3f}', f'{iz:.3f}',
                               f'{tx:.3f}', f'{ty:.3f}', f'{tz:.3f}'])
        self._csv.flush()

        if rng <= self.capture_radius:
            self.captured = True
            self._finish('capture', t)
        elif t >= self.timeout:
            self._finish('timeout', t)

    def _finish(self, outcome, t):
        self.done = True
        self.get_logger().info(
            f"\n==== Interception result ({self.method}) ====\n"
            f"  outcome           : {outcome}\n"
            f"  time_to_intercept : {t:.2f} s\n"
            f"  min_range         : {self.min_range:.2f} m\n"
            f"  capture_radius    : {self.capture_radius} m\n"
            f"  csv               : {self.csv_path}\n"
            f"=============================================")
        try:
            self._csv.flush()
            self._csv.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = InterceptionMetrics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
