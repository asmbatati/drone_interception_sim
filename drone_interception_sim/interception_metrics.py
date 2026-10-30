#!/usr/bin/env python3
"""Method-agnostic interception metrics / benchmark logger.

Works regardless of which controller drives the interceptor (PN strategy, FSM,
Behaviour Tree, or RL): it just watches both drones' odometry and records the
engagement. Use it to compare methods on the same scenario.

Reports (printed on capture/timeout and written to CSV):
  * outcome            capture | miss | timeout. ``miss`` = the interceptor
                       came back down (within ``landed_height`` of its start
                       altitude and moving less than 0.5 m over 1 s, after
                       liftoff) without a capture: a
                       mission head that landed after declaring capture on its
                       *estimate* of the target while the true range stayed
                       above the radius, or a controller that flew into the
                       ground
  * time_to_intercept  seconds from first valid range to capture (NOTE: starts
                       when both odoms are seen, so for full-pipeline runs it
                       includes the arm+takeoff phase)
  * min_range          closest approach (miss distance if no capture)
  * time_engaged       seconds from liftoff (the interceptor first
                       ``liftoff_height`` above its start altitude) to capture.
                       Excludes the time each head spends before leaving the
                       ground (arming, mode requests); includes the climb,
                       which some heads overlap with the pursuit (PN, RL)
                       and others do first (AUTO.TAKEOFF)

CSV columns: t, range, ix, iy, iz, tx, ty, tz

Additionally APPENDS one summary row per run to ``summary_csv_path`` — the
cross-method benchmark table (method, outcome, time_to_intercept, min_range,
capture_radius, trace csv, time_engaged, scenario, trial). Point every run at
the same summary file and the comparison matrix builds itself.

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
        self.declare_parameter('summary_csv_path', '/tmp/interception_results.csv')
        self.declare_parameter('interceptor_spawn', [0.0, 0.0, 0.0])
        self.declare_parameter('target_spawn', [10.0, 0.0, 0.0])
        self.declare_parameter('liftoff_height', 0.5)
        self.declare_parameter('landed_height', 0.3)
        self.declare_parameter('scenario', '')
        self.declare_parameter('trial', -1)

        self.method = self.get_parameter('method').value
        self.capture_radius = self.get_parameter('capture_radius').value
        self.timeout = self.get_parameter('timeout').value
        self.csv_path = self.get_parameter('csv_path').value
        self.summary_csv_path = self.get_parameter('summary_csv_path').value
        self.ispawn = list(self.get_parameter('interceptor_spawn').value)
        self.tspawn = list(self.get_parameter('target_spawn').value)
        # heights are relative to the first interceptor altitude seen: the
        # EKF's ground z drifts by up to ~1 m between flights
        self.liftoff_height = float(self.get_parameter('liftoff_height').value)
        self.landed_height = float(self.get_parameter('landed_height').value)
        self.z0 = None
        self.t_landed = None
        self._recent = []   # (t, x, y) over the last second, for 'stopped'
        self.scenario = str(self.get_parameter('scenario').value)
        self.trial = int(self.get_parameter('trial').value)
        self.t_engage = None

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
        if self.z0 is None:
            self.z0 = iz
        if self.t_engage is None and iz >= self.z0 + self.liftoff_height:
            self.t_engage = t
        self._writer.writerow([f'{t:.3f}', f'{rng:.4f}',
                               f'{ix:.3f}', f'{iy:.3f}', f'{iz:.3f}',
                               f'{tx:.3f}', f'{ty:.3f}', f'{tz:.3f}'])
        self._csv.flush()

        self._recent = [r for r in self._recent if t - r[0] <= 1.0] + [(t, ix, iy)]
        moved = math.hypot(ix - self._recent[0][1], iy - self._recent[0][2])
        if (self.t_engage is not None and iz < self.z0 + self.landed_height
                and moved < 0.5):
            self.t_landed = t if self.t_landed is None else self.t_landed
        else:
            self.t_landed = None

        if rng <= self.capture_radius:
            self.captured = True
            self._finish('capture', t)
        elif self.t_landed is not None and t - self.t_landed >= 1.0:
            self._finish('miss', t)
        elif t >= self.timeout:
            self._finish('timeout', t)

    def _finish(self, outcome, t):
        self.done = True
        self.get_logger().info(
            f"\n==== Interception result ({self.method}) ====\n"
            f"  outcome           : {outcome}\n"
            f"  time_to_intercept : {t:.2f} s\n"
            f"  time_engaged      : {self._engaged(t)}\n"
            f"  min_range         : {self.min_range:.2f} m\n"
            f"  capture_radius    : {self.capture_radius} m\n"
            f"  csv               : {self.csv_path}\n"
            f"=============================================")
        try:
            self._csv.flush()
            self._csv.close()
        except Exception:
            pass
        self._append_summary(outcome, t)

    def _engaged(self, t):
        if self.t_engage is None:
            return ''
        return f'{t - self.t_engage:.2f}'

    def _append_summary(self, outcome, t):
        """Append one row to the cross-method benchmark table."""
        try:
            import os
            new = not os.path.exists(self.summary_csv_path)
            with open(self.summary_csv_path, 'a', newline='') as f:
                w = csv.writer(f)
                if new:
                    w.writerow(['method', 'outcome', 'time_to_intercept_s',
                                'min_range_m', 'capture_radius_m', 'trace_csv',
                                'time_engaged_s', 'scenario', 'trial'])
                w.writerow([self.method, outcome, f'{t:.2f}',
                            f'{self.min_range:.3f}', self.capture_radius,
                            self.csv_path, self._engaged(t), self.scenario,
                            self.trial])
            self.get_logger().info(
                f"summary row appended -> {self.summary_csv_path}")
        except Exception as e:
            self.get_logger().warn(f"could not append summary: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = InterceptionMetrics()
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
