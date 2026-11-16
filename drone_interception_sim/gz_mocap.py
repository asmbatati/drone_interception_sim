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
Motion-capture self-position: Gazebo's pose of each drone, fed to its PX4 EKF.

PX4's gz bridge subscribes to ``/model/<model>/odometry_with_covariance`` and
publishes what arrives there as ``vehicle_visual_odometry``, which EKF2 fuses as
external vision (``EKF2_EV_CTRL``). This node republishes the drones' poses from
``/world/<world>/dynamic_pose/info`` on those topics, so each EKF runs on a
mocap-quality position instead of GPS and barometer. The EKF parameters that make
vision the only position reference are ``MOCAP_PX4_PARAMS``; PX4 reads them at
boot (``run_benchmark.py`` sets them, ``self_position: mocap``).

Each position is published relative to the drone's spawn point in x/y, so its
MAVROS odometry is in its own spawn frame as before, and odometry plus the spawn
offset is Gazebo's position exactly. The stamp is Gazebo's sim time of the pose,
so the EKF fuses each sample at the time it was taken.
"""

import argparse
import math
import time

#: EKF2 parameters for the mocap condition: vision horizontal + vertical position
#: and yaw (no velocity: the poses carry none), vision as the height reference,
#: GPS kept only for the global origin (YAW bit; the sim GPS has no yaw, so it
#: fuses nothing), barometer off, 2 cm position noise.
MOCAP_PX4_PARAMS = {
    'EKF2_EV_CTRL': 11,
    'EKF2_HGT_REF': 3,
    'EKF2_GPS_CTRL': 8,
    'EKF2_BARO_CTRL': 0,
    'EKF2_EVP_NOISE': 0.02,
    'EKF2_EVA_NOISE': 0.05,
    'EKF2_EV_DELAY': 0.0,
}

#: The same parameters at their airframe / PX4 defaults (GPS + baro, the condition
#: of every campaign before the mocap one).
DEFAULT_PX4_PARAMS = {
    'EKF2_EV_CTRL': 15,
    'EKF2_HGT_REF': 0,
    'EKF2_GPS_CTRL': 7,
    'EKF2_BARO_CTRL': 1,
    'EKF2_EVP_NOISE': 0.1,
    'EKF2_EVA_NOISE': 0.1,
    'EKF2_EV_DELAY': 0.0,
}

POS_STD = 0.02  # m, reported position standard deviation
ATT_STD = 0.02  # rad, reported attitude standard deviation


def odometry_message(pose, stamp, offset, odom_cls):
    """Build a gz ``OdometryWithCovariance`` from a gz Pose at ``stamp``, minus ``offset``."""
    m = odom_cls()
    m.header.stamp.CopyFrom(stamp)
    p = m.pose_with_covariance.pose
    p.position.x = pose.position.x - offset[0]
    p.position.y = pose.position.y - offset[1]
    p.position.z = pose.position.z - offset[2]
    p.orientation.CopyFrom(pose.orientation)
    cov = [0.0] * 36
    for i in (0, 7, 14):
        cov[i] = POS_STD**2
    for i in (21, 28, 35):
        cov[i] = ATT_STD**2
    m.pose_with_covariance.covariance.data.extend(cov)
    t = m.twist_with_covariance.twist
    for v in (t.linear, t.angular):  # unknown: NaN, which PX4 reads as no data
        v.x = v.y = v.z = math.nan
    m.twist_with_covariance.covariance.data.extend([math.nan] * 36)
    return m


def parse_model(spec):
    """``name:x,y`` (or ``name:x,y,z``) -> (name, (x, y, z))."""
    name, _, xy = spec.partition(':')
    vals = [float(v) for v in xy.split(',')] if xy else []
    vals += [0.0] * (3 - len(vals))
    return name, tuple(vals[:3])


class GzMocap:
    """Republish Gazebo poses of ``models`` as PX4 external-vision odometry."""

    def __init__(self, world, models, partition=''):
        """Advertise one odometry topic per model and subscribe to the world's poses."""
        from gz.msgs10.odometry_with_covariance_pb2 import OdometryWithCovariance
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node, NodeOptions

        opts = NodeOptions()
        if partition:
            opts.partition = partition
        self._node = Node(opts)
        self._odom_cls = OdometryWithCovariance
        self.models = dict(models)
        self.pubs = {
            name: self._node.advertise(
                f'/model/{name}/odometry_with_covariance', OdometryWithCovariance
            )
            for name in self.models
        }
        self.count = {name: 0 for name in self.models}
        self.topic = f'/world/{world}/dynamic_pose/info'
        self.subscribed = self._node.subscribe(Pose_V, self.topic, self._on_poses)

    def _on_poses(self, msg):
        for pose in msg.pose:
            off = self.models.get(pose.name)
            if off is None:
                continue
            odom = odometry_message(pose, msg.header.stamp, off, self._odom_cls)
            self.pubs[pose.name].publish(odom)
            self.count[pose.name] += 1


def main(argv=None):
    """Run the feed until interrupted, printing the rate per model every 10 s."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument('--world', default='interception')
    ap.add_argument('--partition', default='')
    ap.add_argument(
        '--model',
        action='append',
        required=True,
        help='gz model and its spawn x,y (repeat), e.g. x500_d435_0:0,0',
    )
    args, _ = ap.parse_known_args(argv)  # ignore ROS launch arguments
    mocap = GzMocap(args.world, [parse_model(s) for s in args.model], args.partition)
    if not mocap.subscribed:
        raise SystemExit(f'gz_mocap: could not subscribe to {mocap.topic}')
    print(f'gz_mocap: {mocap.topic} -> {sorted(mocap.pubs)}', flush=True)
    last = dict(mocap.count)
    try:
        while True:
            time.sleep(10.0)
            rates = {n: (mocap.count[n] - last[n]) / 10.0 for n in mocap.count}
            last = dict(mocap.count)
            text = ', '.join(f'{n} {r:.0f}' for n, r in rates.items())
            print(f'gz_mocap rates (Hz): {text}', flush=True)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
