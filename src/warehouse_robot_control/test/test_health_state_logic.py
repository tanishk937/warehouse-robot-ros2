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
Functional tests for the robot health state machine.

These tests cover critical versus non-critical failures and the formal
health-state precedence rules.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'warehouse_robot_control'))

from health_state_logic import (  # noqa: E402
    BOOTING,
    compute_health_state,
    CRITICAL,
    DEGRADED,
    E_STOP,
    HEALTHY,
    WARNING,
)


def _healthy_inputs(**overrides):
    base = {
        'e_stop_active': False,
        'booting': False,
        'critical_sensor_down': False,
        'non_critical_sensor_down': False,
        'battery_critical': False,
        'battery_low': False,
        'localization_lost': False,
        'localization_degraded': False,
    }
    base.update(overrides)
    return base


def test_all_nominal_is_healthy():
    assert compute_health_state(**_healthy_inputs()) == HEALTHY


def test_booting_before_any_data_arrives():
    assert compute_health_state(**_healthy_inputs(booting=True)) == BOOTING


def test_e_stop_wins_over_everything_including_booting():
    assert compute_health_state(**_healthy_inputs(booting=True, e_stop_active=True)) == E_STOP
    assert compute_health_state(**_healthy_inputs(
        e_stop_active=True, critical_sensor_down=True, battery_critical=True,
    )) == E_STOP


def test_critical_sensor_failure_is_critical_not_degraded():
    # This is the exact "critical vs non-critical" behavior required by
    # Section 7: a critical sensor going down must reach CRITICAL.
    assert compute_health_state(**_healthy_inputs(critical_sensor_down=True)) == CRITICAL


def test_non_critical_sensor_failure_is_only_degraded():
    # ...while a non-critical sensor (e.g. IMU) going down must only
    # ever reach DEGRADED -- the robot keeps operating.
    assert compute_health_state(**_healthy_inputs(non_critical_sensor_down=True)) == DEGRADED


def test_battery_critical_is_critical():
    assert compute_health_state(**_healthy_inputs(battery_critical=True)) == CRITICAL


def test_battery_low_is_only_warning():
    assert compute_health_state(**_healthy_inputs(battery_low=True)) == WARNING


def test_localization_completely_lost_is_critical():
    assert compute_health_state(**_healthy_inputs(localization_lost=True)) == CRITICAL


def test_localization_merely_degraded_is_only_warning():
    assert compute_health_state(**_healthy_inputs(localization_degraded=True)) == WARNING


def test_worst_condition_wins_when_several_apply_at_once():
    # A non-critical sensor is down (would be DEGRADED alone) AND the
    # battery is critical (CRITICAL alone) -> overall must be CRITICAL.
    state = compute_health_state(**_healthy_inputs(
        non_critical_sensor_down=True, battery_critical=True,
    ))
    assert state == CRITICAL


def test_warning_beats_degraded_when_both_present():
    state = compute_health_state(**_healthy_inputs(
        non_critical_sensor_down=True, battery_low=True,
    ))
    assert state == WARNING
