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
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


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

        self.frame_id = self.get_parameter('frame_id').value
        self.ns = self.get_parameter('marker_ns').value
        self.color = list(self.get_parameter('color').value)
        self.arm = self.get_parameter('arm_length').value
        self.mesh = self.get_parameter('mesh_resource').value
        self.mesh_scale = self.get_parameter('mesh_scale').value

        self.pub = self.create_publisher(MarkerArray, 'markers', 1)
        period = 1.0 / max(self.get_parameter('rate').value, 1e-3)
        self.create_timer(period, self._publish)

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

    def _mesh_marker(self):
        m = self._base(0, Marker.MESH_RESOURCE)
        m.mesh_resource = self.mesh
        m.mesh_use_embedded_materials = True
        m.scale.x = m.scale.y = m.scale.z = float(self.mesh_scale)
        return [m]

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
        rotors = [(d, d), (-d, -d), (d, -d), (-d, d)]
        for (x, y) in rotors:
            arms.points.append(Point(x=0.0, y=0.0, z=0.0))
            arms.points.append(Point(x=x, y=y, z=0.0))
        markers.append(arms)

        # rotor disks
        for i, (x, y) in enumerate(rotors):
            r = self._base(2 + i, Marker.CYLINDER)
            r.pose.position.x, r.pose.position.y, r.pose.position.z = x, y, 0.02
            r.scale.x = r.scale.y = 0.20
            r.scale.z = 0.02
            r.color.a = 0.85
            markers.append(r)
        return markers

    def _publish(self):
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
