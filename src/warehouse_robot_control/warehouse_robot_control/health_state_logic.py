#!/usr/bin/env python3
# Copyright 2026 Tanishk Patidar
#
# This source code is provided for viewing, evaluation, educational,
# and portfolio purposes. All rights reserved.
#
# See the repository LICENSE file for terms governing copying,
# modification, distribution, and commercial use.

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
