#!/usr/bin/env python3
# Copyright 2026 Tanishk Patidar
#
# This source code is provided for viewing, evaluation, educational,
# and portfolio purposes. All rights reserved.
#
# See the repository LICENSE file for terms governing copying,
# modification, distribution, and commercial use.

from collections import deque
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rosidl_runtime_py.utilities import get_message

from std_msgs.msg import Header
from warehouse_robot_msgs.msg import SensorHeartbeatArray, SensorStatus


class _SensorTracker:
    """Runtime bookkeeping for a single monitored sensor."""

    def __init__(self, name, topic, msg_type_str, criticality, expected_hz, stale_timeout):
        self.name = name
        self.topic = topic
        self.msg_type_str = msg_type_str
        self.criticality = criticality
        self.expected_hz = expected_hz
        self.stale_timeout = stale_timeout
        self.last_msg_time = None
        self.timestamps = deque(maxlen=30)
        self.subscription = None
        self.failure_count = 0
        self._was_healthy = True  # Boot-up without data is not a failure transition.

    def record_message(self):
        now = time.monotonic()
        self.last_msg_time = now
        self.timestamps.append(now)

    def current_frequency(self):
        if len(self.timestamps) < 2:
            return 0.0
        span = self.timestamps[-1] - self.timestamps[0]
        if span <= 0.0:
            return 0.0
        return (len(self.timestamps) - 1) / span

    def seconds_since_last(self):
        if self.last_msg_time is None:
            return float('inf')
        return time.monotonic() - self.last_msg_time

    def is_stale(self):
        return self.seconds_since_last() > self.stale_timeout

    def is_active(self):
        return self.last_msg_time is not None

    def compute_state(self):
        if not self.is_active():
            return SensorStatus.FAILED
        if self.is_stale():
            return SensorStatus.STALE
        return SensorStatus.ONLINE

    def update_failure_count(self, state):
        healthy_now = (state == SensorStatus.ONLINE)
        if self._was_healthy and not healthy_now:
            self.failure_count += 1
        self._was_healthy = healthy_now


class SensorHeartbeatMonitor(Node):

    def __init__(self):
        super().__init__('sensor_heartbeat_monitor')

        self.declare_parameter('check_rate_hz', 5.0)
        self.declare_parameter('sensor_names', [''])

        check_rate_hz = self.get_parameter('check_rate_hz').get_parameter_value().double_value
        sensor_names = self.get_parameter('sensor_names').get_parameter_value().string_array_value
        sensor_names = [n for n in sensor_names if n]

        self._trackers = {}

        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        for name in sensor_names:
            self.declare_parameter(f'{name}.topic', '')
            self.declare_parameter(f'{name}.msg_type', 'sensor_msgs/msg/LaserScan')
            self.declare_parameter(f'{name}.criticality', 'non_critical')
            self.declare_parameter(f'{name}.expected_frequency_hz', 1.0)
            self.declare_parameter(f'{name}.stale_timeout_sec', 2.0)

            topic = self.get_parameter(f'{name}.topic').get_parameter_value().string_value
            msg_type_str = (
                self.get_parameter(f'{name}.msg_type').get_parameter_value().string_value
            )
            criticality = (
                self.get_parameter(f'{name}.criticality').get_parameter_value().string_value
            )
            expected_hz = (
                self.get_parameter(f'{name}.expected_frequency_hz')
                .get_parameter_value().double_value
            )
            stale_timeout = (
                self.get_parameter(f'{name}.stale_timeout_sec').get_parameter_value().double_value
            )

            if not topic:
                self.get_logger().warn(
                    f"Sensor '{name}' has no topic configured, skipping."
                )
                continue

            try:
                msg_class = get_message(msg_type_str)
            except (ValueError, ImportError, AttributeError) as exc:
                self.get_logger().error(
                    f"Sensor '{name}': could not resolve message type "
                    f"'{msg_type_str}' ({exc}). Skipping this sensor."
                )
                continue

            tracker = _SensorTracker(
                name, topic, msg_type_str, criticality, expected_hz, stale_timeout
            )

            def _make_callback(tracker_ref):
                def _cb(_msg):
                    tracker_ref.record_message()
                return _cb

            tracker.subscription = self.create_subscription(
                msg_class, topic, _make_callback(tracker), best_effort_qos
            )
            self._trackers[name] = tracker
            self.get_logger().info(
                f"Monitoring sensor '{name}' on topic '{topic}' "
                f'({criticality}, expected {expected_hz:.1f} Hz, '
                f'type={msg_type_str})'
            )

        self._heartbeat_pub = self.create_publisher(
            SensorHeartbeatArray, '/diagnostics/sensor_heartbeats', reliable_qos
        )
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', reliable_qos)

        period = 1.0 / check_rate_hz if check_rate_hz > 0.0 else 0.2
        self._timer = self.create_timer(period, self._on_timer)

    def _on_timer(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        heartbeat_array = SensorHeartbeatArray()
        heartbeat_array.header = header

        diag_array = DiagnosticArray()
        diag_array.header = header

        for tracker in self._trackers.values():
            state = tracker.compute_state()
            tracker.update_failure_count(state)

            status = SensorStatus()
            status.sensor_name = tracker.name
            status.topic_name = tracker.topic
            status.msg_type = tracker.msg_type_str
            status.criticality = (
                SensorStatus.CRITICAL
                if tracker.criticality == 'critical'
                else SensorStatus.NON_CRITICAL
            )
            status.state = state
            status.is_active = tracker.is_active()
            status.is_stale = tracker.is_stale()
            status.frequency_hz = tracker.current_frequency()
            status.expected_frequency_hz = tracker.expected_hz
            seconds_since = tracker.seconds_since_last()
            status.seconds_since_last_msg = (
                seconds_since if seconds_since != float('inf') else -1.0
            )
            status.failure_count = tracker.failure_count
            heartbeat_array.sensors.append(status)

            diag_status = DiagnosticStatus()
            diag_status.name = f'sensor_heartbeat_monitor: {tracker.name}'
            diag_status.hardware_id = tracker.topic
            if state == SensorStatus.FAILED:
                diag_status.level = DiagnosticStatus.STALE
                diag_status.message = 'No messages received yet'
            elif state == SensorStatus.STALE:
                diag_status.level = (
                    DiagnosticStatus.ERROR if status.criticality == SensorStatus.CRITICAL
                    else DiagnosticStatus.WARN
                )
                diag_status.message = (
                    f'Sensor stale: {status.seconds_since_last_msg:.2f}s since last message '
                    f'(failure_count={tracker.failure_count})'
                )
            else:
                diag_status.level = DiagnosticStatus.OK
                diag_status.message = f'Nominal at {status.frequency_hz:.2f} Hz'

            diag_status.values = [
                KeyValue(key='frequency_hz', value=f'{status.frequency_hz:.2f}'),
                KeyValue(key='expected_frequency_hz', value=f'{status.expected_frequency_hz:.2f}'),
                KeyValue(key='criticality', value=tracker.criticality),
                KeyValue(key='failure_count', value=str(tracker.failure_count)),
            ]
            diag_array.status.append(diag_status)

        self._heartbeat_pub.publish(heartbeat_array)
        self._diag_pub.publish(diag_array)


def main(args=None):
    rclpy.init(args=args)
    node = SensorHeartbeatMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
