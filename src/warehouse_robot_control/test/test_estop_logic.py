# Copyright 2026 Tanishk Patidar
#
# This source code is provided for viewing, evaluation, educational,
# and portfolio purposes. All rights reserved.
#
# See the repository LICENSE file for terms governing copying,
# modification, distribution, and commercial use.


import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'warehouse_robot_control'))

from estop_logic import ACTIVE, CLEAR, decide_transition, RESET_REQUESTED  # noqa: E402


def test_activation_from_clear_is_accepted():
    result = decide_transition(CLEAR, activate=True, confirm_reset=False)
    assert result.new_state == ACTIVE
    assert result.accepted is True


def test_activation_from_active_is_a_no_op_but_still_accepted():
    result = decide_transition(ACTIVE, activate=True, confirm_reset=False)
    assert result.new_state == ACTIVE
    assert result.accepted is True


def test_activation_always_wins_even_during_reset_requested():
    # A fresh trigger during the grace period must win outright.
    result = decide_transition(RESET_REQUESTED, activate=True, confirm_reset=False)
    assert result.new_state == ACTIVE
    assert result.accepted is True


def test_clearing_active_without_confirm_reset_is_rejected():
    # This is the exact unsafe behavior from docs/AUDIT.md #2 that must
    # no longer be possible: activate=false alone must NOT clear an
    # active stop.
    result = decide_transition(ACTIVE, activate=False, confirm_reset=False)
    assert result.new_state == ACTIVE
    assert result.accepted is False


def test_clearing_active_with_confirm_reset_moves_to_reset_requested_not_straight_to_clear():
    result = decide_transition(ACTIVE, activate=False, confirm_reset=True)
    assert result.new_state == RESET_REQUESTED
    assert result.accepted is True


def test_clearing_from_clear_is_idempotent_not_an_error():
    result = decide_transition(CLEAR, activate=False, confirm_reset=False)
    assert result.new_state == CLEAR
    assert result.accepted is True


def test_repeated_confirm_reset_while_pending_is_a_harmless_no_op():
    result = decide_transition(RESET_REQUESTED, activate=False, confirm_reset=True)
    assert result.new_state == RESET_REQUESTED
    assert result.accepted is True


def test_cancelling_a_pending_reset_without_confirmation_is_rejected():
    result = decide_transition(RESET_REQUESTED, activate=False, confirm_reset=False)
    assert result.new_state == RESET_REQUESTED
    assert result.accepted is False
