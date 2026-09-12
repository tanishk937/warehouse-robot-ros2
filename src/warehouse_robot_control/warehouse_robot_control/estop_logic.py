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
Pure emergency-stop state-machine transition logic.

Pure, rclpy-free E-stop state machine transition rules, used by
e_stop_manager.py and unit-tested directly in test/test_estop_logic.py.

States:
    CLEAR            -- robot may move, subject to every other safety layer
    ACTIVE           -- E-stop engaged, /cmd_vel forced to zero
    RESET_REQUESTED  -- an explicit reset was requested and is pending
                        finalization after a grace period, during which
                        a fresh activation immediately wins and cancels it

Design intent (see docs/AUDIT.md #2): activation must always be
allowed instantly, from any state, with no confirmation step, because
that is the safety-critical direction. Clearing must never happen as
a side effect of a generic "set state" call -- it requires
confirm_reset=True, and even then is not final until the grace-period
timer in e_stop_manager.py confirms no new activation arrived.
"""

from dataclasses import dataclass

CLEAR = 0
ACTIVE = 1
RESET_REQUESTED = 2

_STATE_NAMES = {CLEAR: 'CLEAR', ACTIVE: 'ACTIVE', RESET_REQUESTED: 'RESET_REQUESTED'}


def state_name(state: int) -> str:
    return _STATE_NAMES.get(state, f'UNKNOWN({state})')


@dataclass
class TransitionResult:
    new_state: int
    accepted: bool
    message: str


def decide_transition(current_state: int, activate: bool, confirm_reset: bool) -> TransitionResult:
    # Activation always wins, unconditionally, from any state.
    if activate:
        return TransitionResult(ACTIVE, True, 'E-stop ACTIVATED')

    if current_state == CLEAR:
        # Idempotent: asking to clear an already-clear stop is not an error.
        return TransitionResult(CLEAR, True, 'E-stop already CLEAR')

    if current_state == ACTIVE:
        if confirm_reset:
            return TransitionResult(
                RESET_REQUESTED, True,
                'Reset requested — finalizing after grace period if no new activation arrives'
            )
        return TransitionResult(
            ACTIVE, False,
            'E-stop is ACTIVE: clearing requires confirm_reset=true (explicit action required)'
        )

    if current_state == RESET_REQUESTED:
        if confirm_reset:
            return TransitionResult(RESET_REQUESTED, True, 'Reset already pending')
        return TransitionResult(
            RESET_REQUESTED,
            False,
            'A reset is already pending finalization; it cannot be cancelled '
            'by an unconfirmed call',
        )

    return TransitionResult(current_state, False, f'Unknown current state: {current_state}')
