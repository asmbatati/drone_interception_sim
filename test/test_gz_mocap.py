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

"""The motion-capture feed's message and the runner's PX4 parameter parsing."""

import importlib.util
import math
import os
import subprocess
from types import SimpleNamespace

import pytest

from drone_interception_sim.gz_mocap import (
    DEFAULT_PX4_PARAMS,
    MOCAP_PX4_PARAMS,
    odometry_message,
    parse_model,
)


def test_parse_model():
    assert parse_model('x3_uav_1:10,0') == ('x3_uav_1', (10.0, 0.0, 0.0))
    assert parse_model('x500_0:1,2,3') == ('x500_0', (1.0, 2.0, 3.0))


def test_param_sets_cover_the_same_names():
    assert set(MOCAP_PX4_PARAMS) == set(DEFAULT_PX4_PARAMS)
    assert MOCAP_PX4_PARAMS['EKF2_HGT_REF'] == 3  # vision is the height reference
    assert MOCAP_PX4_PARAMS['EKF2_GPS_CTRL'] & 0b11 == 0  # no GPS position


def test_odometry_message_is_in_the_spawn_frame():
    msgs = pytest.importorskip('gz.msgs10.pose_pb2')
    odom = pytest.importorskip('gz.msgs10.odometry_with_covariance_pb2')
    time_pb2 = pytest.importorskip('gz.msgs10.time_pb2')
    pose = msgs.Pose()
    pose.position.x, pose.position.y, pose.position.z = 12.5, -1.0, 3.0
    pose.orientation.w = 1.0
    stamp = time_pb2.Time(sec=42, nsec=5)
    m = odometry_message(pose, stamp, (10.0, 0.0, 0.0), odom.OdometryWithCovariance)
    p = m.pose_with_covariance.pose.position
    assert (p.x, p.y, p.z) == (2.5, -1.0, 3.0)
    assert (m.header.stamp.sec, m.header.stamp.nsec) == (42, 5)  # Gazebo's sim time
    cov = m.pose_with_covariance.covariance.data
    assert cov[0] == cov[7] == cov[14] > 0
    assert math.isnan(m.twist_with_covariance.twist.linear.x)  # no velocity sent


def _runner_module():
    path = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'run_benchmark.py')
    spec = importlib.util.spec_from_file_location('run_benchmark', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_px4_param_parses_the_show_line(monkeypatch):
    rb = _runner_module()
    out = 'Symbols: x = used, + = saved, * = unsaved\nx + EKF2_GPS_CTRL [365,588] : 8\n'
    monkeypatch.setattr(
        subprocess, 'run', lambda *a, **k: SimpleNamespace(stdout=out, stderr='', returncode=0)
    )
    r = rb.Runner.__new__(rb.Runner)
    r.px4_dir = '/nonexistent'
    assert r.px4_param(0, 'EKF2_GPS_CTRL') == 8.0
    assert r.px4_param(0, 'EKF2_EV_CTRL') is None
