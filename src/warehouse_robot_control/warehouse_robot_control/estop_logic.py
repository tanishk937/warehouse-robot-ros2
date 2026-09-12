#!/usr/bin/env python3
# Copyright 2026 Tanishk Patidar
#
# This source code is provided for viewing, evaluation, educational,
# and portfolio purposes. All rights reserved.
#
# See the repository LICENSE file for terms governing copying,
# modification, distribution, and commercial use.

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
