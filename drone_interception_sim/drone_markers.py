#!/usr/bin/env python3
"""Publish RViz markers for a drone, attached to its base_link TF frame.

The drones are spawned from PX4 SDF (no ROS robot_description / URDF), so RViz
has nothing to render for the bodies. This node fills that gap by publishing a
MarkerArray parented to ``frame_id`` (e.g. ``interceptor/base_link``); the
markers then move with the drone through TF - no URDF needed.

By default it draws a simple geometric quadcopter (body + arms + rotor disks),
which works regardless of mesh availability. Set ``mesh_resource`` (a
file:// or package:// URI) to render an actual mesh instead.
"""
import math

from geometry_msgs.msg import Point
from mavros_msgs.msg import State, VfrHud
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from visualization_msgs.msg import Marker, MarkerArray


def _quat_from_rpy(r, p, y):
    """SDF roll-pitch-yaw (XYZ intrinsic) -> (x, y, z, w) quaternion."""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _qconj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def _qrot(q, v):
    """Rotate vector v (3-tuple) by quaternion q."""
    qv = (v[0], v[1], v[2], 0.0)
    rx, ry, rz, _ = _qmul(_qmul(q, qv), _qconj(q))
    return (rx, ry, rz)


class DroneMarkers(Node):
    """Publish a quadcopter MarkerArray on the drone's base_link frame."""

    def __init__(self):
        super().__init__('drone_markers')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('marker_ns', 'drone')
        self.declare_parameter('color', [0.1, 0.4, 1.0])   # RGB 0..1
        self.declare_parameter('arm_length', 0.25)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('mesh_resource', '')        # file://... to use a mesh
        self.declare_parameter('mesh_scale', 1.0)
        # Propeller animation driven by the real PX4 throttle (VFR_HUD).
        self.declare_parameter('max_spin_rate', 80.0)      # rad/s at full throttle
        self.declare_parameter('idle_spin_rate', 12.0)     # rad/s when armed, ~0 throttle
        # Propeller overlay (the body mesh has no props of its own).
        self.declare_parameter('show_props', True)
        self.declare_parameter('prop_z', 0.10)             # blade height above base_link
        self.declare_parameter('prop_len', 0.22)           # blade span
        # Real per-rotor propeller meshes rendered at their true SDF poses. When
        # rotor_meshes is set, each rotor is a spinning MESH_RESOURCE; otherwise
        # geometric blades are used. Poses are flat [x,y,z,roll,pitch,yaw] groups
        # straight from the model SDF, so the node reproduces Gazebo exactly:
        #   marker = T(base_link<-link) . Rz(spin) . T(visual-in-link)
        #   - rotor_link_poses : each rotor LINK pose in the MODEL frame (6/rotor)
        #   - rotor_visual_poses: each prop VISUAL pose within its link (6/rotor)
        #   - base_pose        : base_link pose in the MODEL frame (6)
        #   - rotor_dirs       : +1 CCW / -1 CW per rotor
        # dynamic_typing: an empty-list default would otherwise be inferred as
        # BYTE_ARRAY and reject the STRING/DOUBLE arrays the launch passes.
        dyn = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('rotor_meshes', [], dyn)
        self.declare_parameter('rotor_link_poses', [], dyn)
        self.declare_parameter('rotor_visual_poses', [], dyn)
        self.declare_parameter('base_pose', [0.0] * 6, dyn)
        self.declare_parameter('rotor_dirs', [], dyn)
        # Body mesh visual pose relative to base_link (e.g. x500 = 0,0,.025,0,0,pi).
        self.declare_parameter('body_pose', [0.0] * 6, dyn)

        self.frame_id = self.get_parameter('frame_id').value
        self.ns = self.get_parameter('marker_ns').value
        self.color = list(self.get_parameter('color').value)
        self.arm = self.get_parameter('arm_length').value
        self.mesh = self.get_parameter('mesh_resource').value
        self.mesh_scale = self.get_parameter('mesh_scale').value
        self.max_spin = self.get_parameter('max_spin_rate').value
        self.idle_spin = self.get_parameter('idle_spin_rate').value
        self.show_props = self.get_parameter('show_props').value
        self.prop_z = self.get_parameter('prop_z').value
        self.prop_len = self.get_parameter('prop_len').value
        self.rotor_meshes = list(self.get_parameter('rotor_meshes').value)
        self.rotor_dirs = list(self.get_parameter('rotor_dirs').value)
        link_poses = list(self.get_parameter('rotor_link_poses').value)
        vis_poses = list(self.get_parameter('rotor_visual_poses').value)
        base = list(self.get_parameter('base_pose').value) or [0.0] * 6
        self._rotors = self._build_rotor_transforms(base, link_poses, vis_poses)
        self.body_pose = list(self.get_parameter('body_pose').value) or [0.0] * 6

        # Real flight state for the propeller spin.
        self.armed = False
        self.throttle = 0.0          # 0..1 from VFR_HUD
        self.spin_angle = 0.0
        self._last_t = None
        state_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE,
                               history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(State, 'mavros/state', self._state_cb, state_qos)
        self.create_subscription(VfrHud, 'mavros/vfr_hud', self._vfr_cb,
                                 qos_profile_sensor_data)

        self.pub = self.create_publisher(MarkerArray, 'markers', 1)
        period = 1.0 / max(self.get_parameter('rate').value, 1e-3)
        self.create_timer(period, self._publish)

    def _state_cb(self, msg):
        self.armed = msg.armed

    def _vfr_cb(self, msg):
        # VFR_HUD.throttle is 0..100 (percent) in MAVLink/mavros.
        self.throttle = max(0.0, min(1.0, msg.throttle / 100.0))

    def _base(self, marker_id, mtype):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = self.ns
        m.id = marker_id
        m.type = mtype
        m.action = Marker.ADD
        m.color.r, m.color.g, m.color.b = (float(c) for c in self.color[:3])
        m.color.a = 1.0
        m.pose.orientation.w = 1.0
        return m

    def _build_rotor_transforms(self, base, link_poses, vis_poses):
        """Precompute each rotor's static transforms in the base_link frame.

        Returns a list of (p_link, q_link, p_vis, q_vis, spin_dir) where
        (p_link, q_link) = T(base_link<-link) = inv(base) . link_in_model, and
        (p_vis, q_vis) = the prop visual pose within its link. Per frame the
        marker pose is then T_link . Rz(spin) . T_visual.
        """
        rotors = []
        if not link_poses:
            return rotors
        bp = (base[0], base[1], base[2])
        bq_inv = _qconj(_quat_from_rpy(base[3], base[4], base[5]))
        n = len(link_poses) // 6
        for i in range(n):
            lx, ly, lz, lr, lp, lyaw = link_poses[i * 6:i * 6 + 6]
            q_link = _qmul(bq_inv, _quat_from_rpy(lr, lp, lyaw))
            p_link = _qrot(bq_inv, (lx - bp[0], ly - bp[1], lz - bp[2]))
            if vis_poses and len(vis_poses) >= (i + 1) * 6:
                vx, vy, vz, vr, vp, vyaw = vis_poses[i * 6:i * 6 + 6]
            else:
                vx = vy = vz = vr = vp = vyaw = 0.0
            q_vis = _quat_from_rpy(vr, vp, vyaw)
            d = self.rotor_dirs[i] if i < len(self.rotor_dirs) else 1.0
            rotors.append((p_link, q_link, (vx, vy, vz), q_vis, d))
        return rotors

    def _prop_markers(self, z):
        """The spinning propellers.

        With real rotor meshes configured, each rotor is a MESH_RESOURCE placed
        by T(base_link<-link) . Rz(spin) . T(visual) so it reproduces Gazebo and
        spins about its true hub; otherwise a geometric blade bar is used. The
        spin angle is the real-throttle-driven value.
        """
        markers = []
        if self.rotor_meshes and self._rotors:
            for i, uri in enumerate(self.rotor_meshes):
                if i >= len(self._rotors):
                    break
                p_link, q_link, p_vis, q_vis, d = self._rotors[i]
                s = d * self.spin_angle
                qs = (0.0, 0.0, math.sin(s / 2.0), math.cos(s / 2.0))
                q_ls = _qmul(q_link, qs)
                q_marker = _qmul(q_ls, q_vis)
                off = _qrot(q_ls, p_vis)
                r = self._base(2 + i, Marker.MESH_RESOURCE)
                r.mesh_resource = uri
                r.mesh_use_embedded_materials = True   # .dae materials; STL uses color
                r.pose.position.x = p_link[0] + off[0]
                r.pose.position.y = p_link[1] + off[1]
                r.pose.position.z = p_link[2] + off[2]
                r.pose.orientation.x = q_marker[0]
                r.pose.orientation.y = q_marker[1]
                r.pose.orientation.z = q_marker[2]
                r.pose.orientation.w = q_marker[3]
                r.scale.x = r.scale.y = r.scale.z = 1.0
                r.color.r = r.color.g = r.color.b = 0.1   # dark props for STL meshes
                r.color.a = 1.0
                markers.append(r)
            return markers

        # Geometric blade fallback (no real mesh configured).
        d = self.arm / math.sqrt(2.0)
        rotors = [(d, d), (-d, -d), (d, -d), (-d, d)]
        spin_dir = [1.0, 1.0, -1.0, -1.0]
        for i, (x, y) in enumerate(rotors):
            r = self._base(2 + i, Marker.CUBE)
            r.pose.position.x, r.pose.position.y, r.pose.position.z = x, y, z
            yaw = spin_dir[i] * self.spin_angle
            r.pose.orientation.z = math.sin(yaw / 2.0)
            r.pose.orientation.w = math.cos(yaw / 2.0)
            r.scale.x, r.scale.y, r.scale.z = self.prop_len, 0.03, 0.01
            r.color.a = 0.95
            markers.append(r)
        return markers

    def _mesh_marker(self):
        m = self._base(0, Marker.MESH_RESOURCE)
        m.mesh_resource = self.mesh
        m.mesh_use_embedded_materials = True
        m.scale.x = m.scale.y = m.scale.z = float(self.mesh_scale)
        # Body mesh visual pose relative to base_link (identity for most models).
        bp = self.body_pose
        m.pose.position.x, m.pose.position.y, m.pose.position.z = bp[0], bp[1], bp[2]
        qx, qy, qz, qw = _quat_from_rpy(bp[3], bp[4], bp[5])
        m.pose.orientation.x, m.pose.orientation.y = qx, qy
        m.pose.orientation.z, m.pose.orientation.w = qz, qw
        markers = [m]
        # The body mesh has no propellers, so overlay spinning prop blades.
        if self.show_props:
            markers += self._prop_markers(self.prop_z)
        return markers

    def _quad_markers(self):
        markers = []
        # body
        body = self._base(0, Marker.CUBE)
        body.scale.x, body.scale.y, body.scale.z = 0.18, 0.18, 0.06
        markers.append(body)

        # arms (X config) as a LINE_LIST: centre -> each rotor
        arms = self._base(1, Marker.LINE_LIST)
        arms.scale.x = 0.03
        d = self.arm / math.sqrt(2.0)
        for (x, y) in [(d, d), (-d, -d), (d, -d), (-d, d)]:
            arms.points.append(Point(x=0.0, y=0.0, z=0.0))
            arms.points.append(Point(x=x, y=y, z=0.0))
        markers.append(arms)

        markers += self._prop_markers(0.02)
        return markers

    def _spin_rate(self):
        """Angular rate (rad/s) for the props, from real armed/throttle state."""
        if not self.armed:
            return 0.0
        return self.idle_spin + (self.max_spin - self.idle_spin) * self.throttle

    def _publish(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_t is not None:
            self.spin_angle = (self.spin_angle + self._spin_rate() *
                               (now - self._last_t)) % (2.0 * math.pi)
        self._last_t = now
        markers = self._mesh_marker() if self.mesh else self._quad_markers()
        self.pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = DroneMarkers()
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
