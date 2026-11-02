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
Capture decision for the FSM head.

The estimated range must stay inside the radius for a short time, on fresh
estimates.

The FSM sees the target only through the KF estimate, which is off by up to
about 0.15 m near capture. A single crossing is not capture: on 1 Nov two
mpc_12state trials declared it at an estimated 0.97 and 0.99 m while the true
range was 1.12 and 1.02 m, and the return to launch started from outside the
radius. A stale estimate must not count either: an interceptor flying to where
the target was would otherwise declare capture on a track it no longer has.

Pure Python (no ROS), so it is unit-tested directly.
"""


class CaptureGate:
    """Holds the time since the estimated range last entered the radius."""

    def __init__(self, radius, confirm):
        """Capture needs the range within `radius` for `confirm` seconds."""
        self.radius = float(radius)
        self.confirm = float(confirm)
        self._since = None

    def reset(self):
        """Forget the time spent inside the radius."""
        self._since = None

    def update(self, rng, t, fresh):
        """
        Return True once `rng` has stayed inside the radius until time `t`.

        `fresh` says whether the estimate behind `rng` is current.
        """
        if not fresh or rng > self.radius:
            self._since = None
            return False
        if self._since is None:
            self._since = t
        return t - self._since >= self.confirm
