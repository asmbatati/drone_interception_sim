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

"""Unit tests for the benchmark runner's Gazebo true-range check."""

import csv
import importlib.util
import os

import pytest

_SPEC = importlib.util.spec_from_file_location(
    'run_benchmark',
    os.path.join(os.path.dirname(__file__), '..', 'scripts', 'run_benchmark.py'),
)
rb = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(rb)

METRICS_HEADER = list(rb.SUMMARY_COLUMNS)


def _row(outcome, radius='1.0', trial='0'):
    return ['m', outcome, '30.00', '0.800', radius, 'trace.csv', '25.00', 'circle', trial]


def _read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def test_min_range_stops_at_the_row_time():
    samples = [(10.0, 5.0), (11.0, 0.7), (12.0, 0.3)]
    assert rb.min_range_until(samples, 11.0) == 0.7
    assert rb.min_range_until(samples, 9.0) is None
    assert rb.min_range_until([], 11.0) is None


def test_true_outcome():
    assert rb.true_outcome('capture', 0.9, 1.0) == 'capture'
    assert rb.true_outcome('capture', 1.4, 1.0) == 'false_capture'
    assert rb.true_outcome('miss', 0.5, 1.0) == 'miss'
    assert rb.true_outcome('capture', None, 1.0) == ''


def test_annotate_extends_a_metrics_file_and_pads_old_rows(tmp_path):
    path = tmp_path / 'results.csv'
    with open(path, 'w', newline='') as f:
        csv.writer(f).writerows([METRICS_HEADER, _row('miss'), _row('capture', trial='1')])
    assert rb.annotate_last(path, 1.3) == 'false_capture'
    rows = _read(path)
    assert list(rows[0]) == METRICS_HEADER + list(rb.TRUE_COLUMNS)
    assert rows[0]['true_min_range_m'] == '' and rows[0]['true_outcome'] == ''
    assert rows[1]['true_min_range_m'] == '1.300'
    assert rows[1]['outcome'] == 'capture' and rows[1]['true_outcome'] == 'false_capture'

    # the metrics node appends its 9-column row to the extended file
    with open(path, 'a', newline='') as f:
        csv.writer(f).writerow(_row('capture', trial='2'))
    assert rb.annotate_last(path, 0.6) == 'capture'
    rows = _read(path)
    assert [r['true_outcome'] for r in rows] == ['', 'false_capture', 'capture']
    with open(path) as f:
        assert {len(r) for r in csv.reader(f)} == {len(METRICS_HEADER) + 2}


def test_annotate_without_poses_leaves_the_row_unverified(tmp_path):
    path = tmp_path / 'results.csv'
    with open(path, 'w', newline='') as f:
        csv.writer(f).writerows([METRICS_HEADER, _row('capture')])
    assert rb.annotate_last(path, None) == ''
    assert _read(path)[0]['true_min_range_m'] == ''


def _pose_v(entries):
    pose_v_pb2 = pytest.importorskip('gz.msgs10.pose_v_pb2')
    msg = pose_v_pb2.Pose_V()
    for name, (x, y, z) in entries:
        p = msg.pose.add()
        p.name = name
        p.position.x, p.position.y, p.position.z = x, y, z
    return msg


def _poses():
    g = rb.GzPoses.__new__(rb.GzPoses)  # no gz node: feed messages by hand
    g.models = rb.GZ_MODELS
    g.samples = []
    return g


def test_pose_callback_ranges_the_two_models_only():
    g = _poses()
    g._on_poses(_pose_v([('x500_d435_0', (0, 0, 4)), ('base_link', (9, 9, 9)),
                         ('x3_uav_1', (3, 4, 4))]))
    g._on_poses(_pose_v([('x500_d435_0', (0, 0, 4))]))  # target missing: skipped
    assert len(g.samples) == 1
    assert g.samples[0][1] == pytest.approx(5.0)


def test_trace_is_decimated_and_cut(tmp_path):
    g = _poses()
    g.samples = [(100.0 + 0.02 * i, 1.0 + i, 0, 0, 0, 0, 0, 0) for i in range(100)]
    path = tmp_path / 'gz.csv'
    g.write_trace(path, t_end=101.0)
    rows = _read(path)
    assert 9 <= len(rows) <= 11
    assert float(rows[-1]['wall_time']) <= 101.0
