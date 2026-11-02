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

"""Unit tests for the FSM head's capture gate (times are binary fractions)."""

from drone_interception_sim.capture_gate import CaptureGate


def test_single_crossing_is_not_capture():
    g = CaptureGate(radius=1.0, confirm=0.5)
    assert not g.update(0.97, 10.0, True)
    assert not g.update(0.99, 10.25, True)


def test_holds_inside_for_the_confirm_time():
    g = CaptureGate(radius=1.0, confirm=0.5)
    assert not g.update(0.9, 10.0, True)
    assert not g.update(0.8, 10.25, True)
    assert g.update(0.7, 10.5, True)


def test_leaving_the_radius_restarts_the_clock():
    g = CaptureGate(radius=1.0, confirm=0.5)
    g.update(0.9, 10.0, True)
    g.update(1.25, 10.25, True)
    assert not g.update(0.9, 10.5, True)
    assert not g.update(0.9, 10.75, True)
    assert g.update(0.9, 11.0, True)


def test_stale_estimate_never_captures():
    g = CaptureGate(radius=1.0, confirm=0.5)
    g.update(0.5, 10.0, True)
    assert not g.update(0.5, 11.0, False)
    # losing the track also restarts the clock
    assert not g.update(0.5, 11.25, True)
    assert g.update(0.5, 11.75, True)


def test_reset():
    g = CaptureGate(radius=1.0, confirm=0.0)
    assert g.update(0.5, 1.0, True)
    g.reset()
    assert g.update(0.5, 2.0, True)
