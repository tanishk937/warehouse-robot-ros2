# Copyright 2026 Tanishk Patidar
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
Functional tests for pure safety decision logic.

Real, runnable functional tests for the safety-critical decision logic
in safety_logic.py. These need no ROS 2 install, no rclpy, and no
Gazebo -- run with plain `pytest` from warehouse_robot_control/.

These directly exercise the acceptance criteria in Section 37 of the
upgrade brief for obstacle avoidance and battery-derated speed.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'warehouse_robot_control'))

from safety_logic import (  # noqa: E402
    battery_speed_scale,
    decide_motion,
    is_scan_geometrically_valid,
    sector_min_range,
)

NORMAL, LOW, CRITICAL = 0, 1, 2


def test_scan_validity_rejects_zero_ranges():
    assert is_scan_geometrically_valid(0, 0.01) is False


def test_scan_validity_rejects_zero_angle_increment():
    # This is the exact malformed-scan edge case called out in
    # docs/AUDIT.md #11: a zero increment would otherwise silently
    # collapse every reading onto one angle.
    assert is_scan_geometrically_valid(360, 0.0) is False


def test_scan_validity_rejects_negative_angle_increment():
    assert is_scan_geometrically_valid(360, -0.01) is False


def test_scan_validity_accepts_normal_scan():
    assert is_scan_geometrically_valid(360, math.radians(1.0)) is True


def test_sector_min_range_ignores_nan_and_inf():
    ranges = [float('nan'), float('inf'), 0.5, 10.0]
    result = sector_min_range(
        ranges, angle_min=-0.2, angle_increment=0.1,
        range_min=0.1, range_max=10.0,
        center_angle=0.0, half_width_rad=0.5,
    )
    assert result == 0.5


def test_sector_min_range_ignores_out_of_range_values():
    # range_max is 10.0, so 50.0 must never win even though it's the
    # numeric minimum-looking value in a naive scan of a huge open room.
    ranges = [50.0, 50.0, 50.0, 50.0]
    result = sector_min_range(
        ranges, angle_min=-0.2, angle_increment=0.1,
        range_min=0.1, range_max=10.0,
        center_angle=0.0, half_width_rad=0.5,
    )
    assert result == float('inf')


def test_sector_min_range_no_reading_in_sector_returns_inf():
    ranges = [1.0, 1.0]
    result = sector_min_range(
        ranges, angle_min=10.0, angle_increment=0.1,  # sector nowhere near these angles
        range_min=0.1, range_max=10.0,
        center_angle=0.0, half_width_rad=0.1,
    )
    assert result == float('inf')


def test_battery_speed_scale_normal_is_full_speed():
    assert battery_speed_scale(NORMAL, NORMAL, LOW, CRITICAL, 0.4) == 1.0


def test_battery_speed_scale_low_is_derated():
    assert battery_speed_scale(LOW, NORMAL, LOW, CRITICAL, 0.4) == 0.4


def test_decide_motion_clear_path_cruises_at_full_speed():
    decision = decide_motion(
        front_min=5.0, left_min=5.0, right_min=5.0,
        stop_distance=0.45, slow_distance=1.0,
        cruise_speed=0.3, avoid_angular_speed=0.6, speed_scale=1.0,
    )
    assert decision.state == 'CRUISING'
    assert decision.linear_x == 0.3
    assert decision.angular_z == 0.0


def test_decide_motion_obstacle_too_close_stops_and_turns_away_from_more_open_side():
    # Obstacle close in front, more room on the left than the right ->
    # must turn toward the left (positive angular.z), never drive forward.
    decision = decide_motion(
        front_min=0.2, left_min=3.0, right_min=0.3,
        stop_distance=0.45, slow_distance=1.0,
        cruise_speed=0.3, avoid_angular_speed=0.6, speed_scale=1.0,
    )
    assert decision.state == 'OBSTACLE_STOP'
    assert decision.linear_x == 0.0
    assert decision.angular_z == 0.6


def test_decide_motion_turns_away_from_the_closer_side_not_towards_it():
    # More room on the right than the left -> must turn right (negative).
    decision = decide_motion(
        front_min=0.2, left_min=0.3, right_min=3.0,
        stop_distance=0.45, slow_distance=1.0,
        cruise_speed=0.3, avoid_angular_speed=0.6, speed_scale=1.0,
    )
    assert decision.angular_z == -0.6


def test_decide_motion_approaching_obstacle_slows_down_proportionally():
    decision = decide_motion(
        front_min=0.7, left_min=5.0, right_min=5.0,  # halfway between stop(0.45) and slow(1.0)
        stop_distance=0.45, slow_distance=1.0,
        cruise_speed=1.0, avoid_angular_speed=0.6, speed_scale=1.0,
    )
    assert decision.state == 'SLOW'
    assert 0.0 < decision.linear_x < 1.0


def test_decide_motion_low_battery_derates_cruise_speed():
    decision = decide_motion(
        front_min=5.0, left_min=5.0, right_min=5.0,
        stop_distance=0.45, slow_distance=1.0,
        cruise_speed=1.0, avoid_angular_speed=0.6, speed_scale=0.4,
        base_state='DEGRADED_LOW_BATTERY',
    )
    assert decision.linear_x == 0.4
    assert decision.state == 'DEGRADED_LOW_BATTERY'


def test_decide_motion_never_commands_forward_motion_when_stopped():
    # Regression guard: whatever state comes back for a too-close
    # obstacle, linear.x must be exactly zero -- never a small positive
    # "creeping" value.
    for front in (0.0, 0.01, 0.1, 0.44):
        decision = decide_motion(
            front_min=front, left_min=5.0, right_min=5.0,
            stop_distance=0.45, slow_distance=1.0,
            cruise_speed=1.0, avoid_angular_speed=0.6, speed_scale=1.0,
        )
        assert decision.linear_x == 0.0
