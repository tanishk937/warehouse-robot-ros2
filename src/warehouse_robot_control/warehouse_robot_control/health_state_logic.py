#!/usr/bin/env python3
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
Pure robot health state-machine logic.

Pure, rclpy-free implementation of the formal robot health state
machine (Section 18 of the upgrade brief), unit-tested directly in
test/test_health_state_logic.py.

States (mirrors warehouse_robot_msgs/msg/RobotHealth constants):
    BOOTING   -- not enough data has arrived yet to judge health
    HEALTHY   -- no faults of any kind
    DEGRADED  -- a non-critical sensor is stale/failed; robot fully operational
    WARNING   -- battery LOW, or localization is degraded but not lost
    CRITICAL  -- a critical sensor is stale/failed, battery CRITICAL, or
                 localization is completely lost (stale)
    E_STOP    -- E-stop is active (always wins, even while booting)
    SHUTDOWN  -- not computed here; set only by external orchestration
                 on deliberate node shutdown.

Precedence when multiple conditions are true simultaneously (worst wins):
    E_STOP > CRITICAL > WARNING > DEGRADED > HEALTHY
BOOTING is only reachable when nothing else applies and insufficient
data has arrived.
"""

BOOTING = 0
HEALTHY = 1
DEGRADED = 2
WARNING = 3
CRITICAL = 4
E_STOP = 5
SHUTDOWN = 6

_STATE_NAMES = {
    BOOTING: 'BOOTING', HEALTHY: 'HEALTHY', DEGRADED: 'DEGRADED',
    WARNING: 'WARNING', CRITICAL: 'CRITICAL', E_STOP: 'E_STOP', SHUTDOWN: 'SHUTDOWN',
}


def state_name(state: int) -> str:
    return _STATE_NAMES.get(state, f'UNKNOWN({state})')


def compute_health_state(
    e_stop_active: bool,
    booting: bool,
    critical_sensor_down: bool,
    non_critical_sensor_down: bool,
    battery_critical: bool,
    battery_low: bool,
    localization_lost: bool,
    localization_degraded: bool,
) -> int:
    """
    Compute health from boolean inputs using the defined precedence.

    The caller derives these booleans from live data; this function only
    applies the precedence rules.
    """
    if e_stop_active:
        return E_STOP

    if booting:
        return BOOTING

    if critical_sensor_down or battery_critical or localization_lost:
        return CRITICAL

    if battery_low or localization_degraded:
        return WARNING

    if non_critical_sensor_down:
        return DEGRADED

    return HEALTHY
