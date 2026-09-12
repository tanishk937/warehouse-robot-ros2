#!/usr/bin/env python3
# Copyright 2026 Tanishk Patidar
#
# This source code is provided for viewing, evaluation, educational,
# and portfolio purposes. All rights reserved.
#
# See the repository LICENSE file for terms governing copying,
# modification, distribution, and commercial use.


from dataclasses import dataclass
import math


def is_scan_geometrically_valid(num_ranges: int, angle_increment: float) -> bool:
    """
    Return whether a LaserScan has valid geometry.

    Reject zero ranges and non-positive angle increments rather than
    guessing about malformed scan geometry.
    """
    return num_ranges > 0 and math.isfinite(angle_increment) and angle_increment > 0.0


def sector_min_range(
    ranges, angle_min, angle_increment, range_min, range_max,
    center_angle, half_width_rad
):
    """
    Return the minimum valid reading within an angular sector.

    Return ``float('inf')`` when no valid reading falls in the sector;
    callers must first verify that the sensor itself is fresh.
    """
    min_range = float('inf')
    angle = angle_min
    lo = center_angle - half_width_rad
    hi = center_angle + half_width_rad
    for r in ranges:
        if lo <= angle <= hi and math.isfinite(r) and range_min <= r <= range_max:
            if r < min_range:
                min_range = r
        angle += angle_increment
    return min_range


def battery_speed_scale(
    battery_state, battery_normal, battery_low, battery_critical,
    low_scale: float
) -> float:
    """
    Return the speed scale for the current battery state.

    NORMAL returns 1.0; LOW returns ``low_scale``; CRITICAL is handled by
    the caller before this function is consulted.
    """
    if battery_state == battery_low:
        return low_scale
    return 1.0


@dataclass
class MotionDecision:
    linear_x: float
    angular_z: float
    state: str


def decide_motion(
    front_min, left_min, right_min,
    stop_distance, slow_distance,
    cruise_speed, avoid_angular_speed,
    speed_scale, base_state='CRUISING'
) -> MotionDecision:
    """
    Return a reactive obstacle-avoidance decision for one control cycle.

    The caller handles E-stop, stale-sensor, and critical-battery stops
    before this function is called.
    """
    if front_min < stop_distance:
        angular = avoid_angular_speed if left_min >= right_min else -avoid_angular_speed
        return MotionDecision(0.0, angular, 'OBSTACLE_STOP')

    if front_min < slow_distance:
        span = slow_distance - stop_distance
        slow_fraction = (front_min - stop_distance) / span if span > 0 else 1.0
        linear = cruise_speed * max(0.1, slow_fraction) * speed_scale
        angular = 0.15 if left_min >= right_min else -0.15
        return MotionDecision(linear, angular, 'SLOW')

    return MotionDecision(cruise_speed * speed_scale, 0.0, base_state)
