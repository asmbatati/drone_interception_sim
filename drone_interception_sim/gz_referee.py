# Copyright 2026 Abdulrahman S. Al-Batati
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Gazebo's own poses of both vehicles, to referee an engagement.

MAVROS odometry puts the interceptor up to about 1 m off its true position (its
EKF height), so a capture judged on odometry is often 1-2 m short of one
(benchmark of 3-4 Nov 2026: 30 of 152 declared captures within 1 m by Gazebo).
This reads ``/world/<world>/dynamic_pose/info`` over gz-transport in the
``GZ_PARTITION`` of the process; no ROS bridge is needed.
"""

import time

#: gz model names PX4 spawns (``<model>_<instance>``): interceptor, target
GZ_MODELS = ('x500_d435_0', 'x3_uav_1')


def pose_positions(msg, models):
    """Return the ``(x, y, z)`` of each of ``models`` in a Pose_V message, or None."""
    pos = {p.name: (p.position.x, p.position.y, p.position.z) for p in msg.pose}
    if any(m not in pos for m in models):
        return None
    return [pos[m] for m in models]


class GzReferee:
    """Latest Gazebo positions of the interceptor and the target."""

    def __init__(self, world='interception', models=GZ_MODELS, partition=''):
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node, NodeOptions

        opts = NodeOptions()
        if partition:
            opts.partition = partition
        self._node = Node(opts)
        self.models = tuple(models)
        self.topic = f'/world/{world}/dynamic_pose/info'
        self._latest = None  # (monotonic time, interceptor xyz, target xyz)
        self.subscribed = self._node.subscribe(Pose_V, self.topic, self._on_poses)

    def _on_poses(self, msg):
        p = pose_positions(msg, self.models)
        if p is not None:
            self._latest = (time.monotonic(), p[0], p[1])

    def latest(self, max_age=0.5):
        """Return ``(interceptor_xyz, target_xyz)``, or None if older than ``max_age`` s."""
        s = self._latest
        if s is None or time.monotonic() - s[0] > max_age:
            return None
        return s[1], s[2]
