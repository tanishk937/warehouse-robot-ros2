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
Emergency-stop state manager for the warehouse robot.

Central authority for the robot's emergency-stop state, implementing
the CLEAR / ACTIVE / RESET_REQUESTED state machine defined in
estop_logic.py (see docs/AUDIT.md #2 for why the previous
unconditional activate=false clear was unsafe).

Any client -- a remote dashboard, a physical button bridge, or another
node -- can activate or clear the stop through either:
    * the /e_stop/set service (warehouse_robot_msgs/srv/SetEStop), or
    * a std_msgs/Bool published on /e_stop/remote_trigger (activation only;
      clearing an E-stop is deliberately NOT possible from this topic --
      see _on_remote_trigger).

The resulting boolean state is republished as a latched
(TRANSIENT_LOCAL) std_msgs/Bool on /e_stop/status, so any node started
*after* the stop was triggered still immediately receives the current
state on subscription -- this is exactly the topic safety_controller
listens to in order to zero /cmd_vel the instant a stop is requested.
The richer 3-state machine value is published on /e_stop/state_machine
(std_msgs/UInt8) for dashboard/debug visibility.

Publishes:
    /e_stop/status          (std_msgs/Bool,   RELIABLE + TRANSIENT_LOCAL)
    /e_stop/state_machine   (std_msgs/UInt8,  RELIABLE + TRANSIENT_LOCAL)
Subscribes:
    /e_stop/remote_trigger  (std_msgs/Bool) -- activation-only, see above
Services:
    /e_stop/set             (warehouse_robot_msgs/srv/SetEStop)
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from std_msgs.msg import Bool, UInt8

from warehouse_robot_control.estop_logic import (
    CLEAR,
    decide_transition,
    RESET_REQUESTED,
    state_name,
)
from warehouse_robot_msgs.srv import SetEStop


class EStopManager(Node):

    def __init__(self):
        super().__init__('e_stop_manager')

        self.declare_parameter('status_publish_rate_hz', 5.0)
        self.declare_parameter('reset_grace_period_sec', 1.0)

        publish_rate_hz = self.get_parameter('status_publish_rate_hz').value
        self._grace_period_sec = self.get_parameter('reset_grace_period_sec').value

        self._lock = threading.Lock()
        self._state = CLEAR
        self._reason = 'none'
        self._pending_reset_timer = None

        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._status_pub = self.create_publisher(Bool, '/e_stop/status', latched_qos)
        self._state_pub = self.create_publisher(UInt8, '/e_stop/state_machine', latched_qos)
        self.create_subscription(Bool, '/e_stop/remote_trigger', self._on_remote_trigger, 10)
        self.create_service(SetEStop, '/e_stop/set', self._on_set_e_stop)

        period = 1.0 / publish_rate_hz if publish_rate_hz > 0 else 0.2
        self.create_timer(period, self._publish_status)

        self._publish_status()
        self.get_logger().info(
            f'[ESTOP] e_stop_manager ready (initial state: CLEAR, '
            f'reset_grace_period_sec={self._grace_period_sec})'
        )

    def _is_active_bool(self) -> bool:
        # RESET_REQUESTED still forces zero velocity until finalized --
        # only CLEAR permits motion.
        return self._state != CLEAR

    def _apply_transition(
        self, activate: bool, confirm_reset: bool, source: str, reason: str = ''
    ):
        with self._lock:
            result = decide_transition(self._state, activate, confirm_reset)
            old_state = self._state
            self._state = result.new_state
            if activate:
                self._reason = reason or 'unspecified'
            elif result.new_state == CLEAR:
                self._reason = 'none'

            # Cancel any previously pending finalize timer -- either a
            # fresh activation superseded it, or we just (re)entered
            # RESET_REQUESTED and need a new one.
            if self._pending_reset_timer is not None:
                self._pending_reset_timer.cancel()
                self._pending_reset_timer = None

            schedule_finalize = (result.new_state == RESET_REQUESTED and result.accepted)

        if old_state != result.new_state:
            self.get_logger().warn(
                f'[ESTOP] {state_name(old_state)} -> {state_name(result.new_state)} '
                f'(source={source}, reason="{self._reason}")'
            )

        if schedule_finalize:
            self._pending_reset_timer = self.create_timer(
                self._grace_period_sec, self._finalize_reset,
            )

        self._publish_status()
        return result

    def _finalize_reset(self):
        with self._lock:
            if self._pending_reset_timer is not None:
                self._pending_reset_timer.cancel()
                self._pending_reset_timer = None
            if self._state != RESET_REQUESTED:
                # A new activation arrived during the grace period; do nothing.
                return
            self._state = CLEAR
            self._reason = 'none'

        self.get_logger().warn(
            '[ESTOP] RESET_REQUESTED -> CLEAR (grace period elapsed, no re-trigger)'
        )
        self._publish_status()

    def _on_remote_trigger(self, msg: Bool):
        if not msg.data:
            # Deliberately not honored: clearing requires the explicit
            # confirm_reset service call, never a bare topic publish.
            self.get_logger().warn(
                '[ESTOP] Ignored attempt to clear E-stop via /e_stop/remote_trigger '
                '(clearing requires the /e_stop/set service with confirm_reset=true)'
            )
            return
        self._apply_transition(True, False, 'remote_trigger_topic', reason='remote_trigger_topic')

    def _on_set_e_stop(self, request: SetEStop.Request, response: SetEStop.Response):
        result = self._apply_transition(
            request.activate, request.confirm_reset, 'service_call', reason=request.reason,
        )
        response.success = result.accepted
        response.message = result.message
        response.state = result.new_state
        return response

    def _publish_status(self):
        with self._lock:
            active = self._is_active_bool()
            state = self._state
        self._status_pub.publish(Bool(data=active))
        self._state_pub.publish(UInt8(data=state))


def main(args=None):
    rclpy.init(args=args)
    node = EStopManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
