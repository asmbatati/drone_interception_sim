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

"""Unit tests for the Gazebo referee's pose handling (no gz server needed)."""

import time

from drone_interception_sim.gz_referee import GZ_MODELS, GzReferee, pose_positions
import pytest


def _pose_v(entries):
    pose_v_pb2 = pytest.importorskip('gz.msgs10.pose_v_pb2')
    msg = pose_v_pb2.Pose_V()
    for name, (x, y, z) in entries:
        p = msg.pose.add()
        p.name = name
        p.position.x, p.position.y, p.position.z = x, y, z
    return msg


def _referee():
    r = GzReferee.__new__(GzReferee)  # no gz node: feed messages by hand
    r.models = GZ_MODELS
    r.topic = '/world/interception/dynamic_pose/info'
    r._latest = None
    return r


def test_positions_of_both_models_in_order():
    msg = _pose_v([('x3_uav_1', (10, 0, 4)), ('base_link', (9, 9, 9)), ('x500_d435_0', (0, 0, 0))])
    assert pose_positions(msg, GZ_MODELS) == [(0, 0, 0), (10, 0, 4)]


def test_a_missing_model_gives_none():
    assert pose_positions(_pose_v([('x500_d435_0', (0, 0, 0))]), GZ_MODELS) is None


def test_latest_holds_the_newest_poses_while_fresh():
    r = _referee()
    assert r.latest() is None
    r._on_poses(_pose_v([('x500_d435_0', (0, 0, 1)), ('x3_uav_1', (3, 4, 1))]))
    assert r.latest() == ((0, 0, 1), (3, 4, 1))
    r._latest = (time.monotonic() - 1.0,) + r._latest[1:]
    assert r.latest(max_age=0.5) is None  # stale: the referee pauses
