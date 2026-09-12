#!/usr/bin/env python3
# Copyright 2026 Tanishk Patidar
#
# This source code is provided for viewing, evaluation, educational,
# and portfolio purposes. All rights reserved.
#
# See the repository LICENSE file for terms governing copying,
# modification, distribution, and commercial use.


import json
import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool, String

from warehouse_robot_control.health_state_logic import (
    BOOTING,
    compute_health_state,
    CRITICAL,
    DEGRADED,
    E_STOP,
    HEALTHY,
    state_name,
    WARNING,
)
from warehouse_robot_msgs.msg import (
    BatteryStatus,
    LocalizationStatus,
    RobotHealth,
    SensorHeartbeatArray,
    SensorStatus,
)
from warehouse_robot_msgs.srv import GetSystemHealth


def _yaw_from_quaternion(q) -> float:
    """
    Convert a geometry_msgs/Quaternion to a yaw angle.

    Use the standard conversion without an external dependency because
    the dashboard only needs the single yaw angle for a ground robot.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class HealthAggregator(Node):

    def __init__(self):
        super().__init__('health_aggregator')

        self.declare_parameter('publish_rate_hz', 2.0)
        publish_rate_hz = self.get_parameter('publish_rate_hz').value

        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._battery = BatteryStatus()
        self._localization = LocalizationStatus()
        self._sensor_heartbeats = SensorHeartbeatArray()
        self._e_stop_active = False
        self._safety_state = 'UNKNOWN'
        self._latest_health = RobotHealth()

        self._have_battery = False
        self._have_sensor_heartbeats = False
        self._previous_health_state = None

        self.create_subscription(BatteryStatus, '/battery/status', self._on_battery, latched_qos)
        self.create_subscription(
            LocalizationStatus, '/localization/status', self._on_localization, reliable_qos
        )
        self.create_subscription(
            SensorHeartbeatArray,
            '/diagnostics/sensor_heartbeats', self._on_sensor_heartbeats, reliable_qos
        )
        self.create_subscription(Bool, '/e_stop/status', self._on_e_stop, latched_qos)
        self.create_subscription(
            String, '/safety_controller/state', self._on_safety_state, reliable_qos
        )

        self._health_pub = self.create_publisher(RobotHealth, '/robot_health', reliable_qos)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', reliable_qos)
        self._dashboard_pub = self.create_publisher(
            String, '/robot_health/dashboard_json', reliable_qos
        )

        self.create_service(GetSystemHealth, '/robot_health/get', self._on_get_health)

        period = 1.0 / publish_rate_hz if publish_rate_hz > 0 else 0.5
        self.create_timer(period, self._on_timer)

        self.get_logger().info('[HEALTH] health_aggregator started.')

    def _on_battery(self, msg):
        self._battery = msg
        self._have_battery = True

    def _on_localization(self, msg):
        self._localization = msg

    def _on_sensor_heartbeats(self, msg):
        self._sensor_heartbeats = msg
        self._have_sensor_heartbeats = True

    def _on_e_stop(self, msg):
        self._e_stop_active = msg.data

    def _on_safety_state(self, msg):
        self._safety_state = msg.data

    def _build_health(self) -> RobotHealth:
        faults = []

        if self._e_stop_active:
            faults.append('E-STOP is active')

        battery_critical = self._battery.state == BatteryStatus.CRITICAL
        battery_low = self._battery.state == BatteryStatus.LOW
        if battery_critical:
            faults.append(f'Battery CRITICAL ({self._battery.percentage:.1f}%)')
        elif battery_low:
            faults.append(f'Battery LOW ({self._battery.percentage:.1f}%)')

        localization_lost = self._localization.is_stale
        localization_degraded = (not self._localization.is_healthy) and not localization_lost
        if localization_lost:
            localization_message = self._localization.status_message or 'no odometry'
            faults.append(f'Localization LOST: {localization_message}')
        elif localization_degraded:
            faults.append(f'Localization degraded: {self._localization.status_message}')

        critical_sensor_down = False
        non_critical_sensor_down = False
        for sensor in self._sensor_heartbeats.sensors:
            if sensor.state != SensorStatus.ONLINE:
                descriptor = (
                    'FAILED (never received)'
                    if sensor.state == SensorStatus.FAILED
                    else 'STALE'
                )
                faults.append(
                    f"Sensor '{sensor.sensor_name}' {descriptor} ({sensor.topic_name}), "
                    f'failure_count={sensor.failure_count}'
                )
                if sensor.criticality == SensorStatus.CRITICAL:
                    critical_sensor_down = True
                else:
                    non_critical_sensor_down = True

        booting = not (self._have_battery and self._have_sensor_heartbeats)

        health_state = compute_health_state(
            e_stop_active=self._e_stop_active,
            booting=booting,
            critical_sensor_down=critical_sensor_down,
            non_critical_sensor_down=non_critical_sensor_down,
            battery_critical=battery_critical,
            battery_low=battery_low,
            localization_lost=localization_lost,
            localization_degraded=localization_degraded,
        )

        if health_state != self._previous_health_state and self._previous_health_state is not None:
            self.get_logger().warn(
                f'[HEALTH] Robot state changed: '
                f'{state_name(self._previous_health_state)} -> {state_name(health_state)}'
            )
        self._previous_health_state = health_state

        health = RobotHealth()
        health.header.stamp = self.get_clock().now().to_msg()
        health.header.frame_id = 'base_link'
        health.health_state = health_state
        health.battery = self._battery
        health.localization = self._localization
        health.sensor_heartbeats = self._sensor_heartbeats
        health.e_stop_active = self._e_stop_active
        health.safety_state = self._safety_state
        health.active_faults = faults

        dashboard = {
            'health_state': state_name(health_state),
            'e_stop_active': self._e_stop_active,
            'safety_controller_state': self._safety_state,
            'battery': {
                'percentage': self._battery.percentage,
                'voltage': self._battery.voltage,
                'current': self._battery.current,
                'state': ['NORMAL', 'LOW', 'CRITICAL', 'CHARGING'][self._battery.state],
                'is_simulated': self._battery.is_simulated,
                'estimated_remaining_minutes': self._battery.estimated_remaining_minutes,
            },
            'localization': {
                'is_healthy': self._localization.is_healthy,
                'is_stale': self._localization.is_stale,
                'source': self._localization.source,
                'pose': {
                    'x': self._localization.pose.position.x,
                    'y': self._localization.pose.position.y,
                    'yaw': _yaw_from_quaternion(self._localization.pose.orientation),
                },
                'linear_speed': self._localization.linear_speed,
                'angular_speed': self._localization.angular_speed,
                'message': self._localization.status_message,
            },
            'sensors': [
                {
                    'name': s.sensor_name,
                    'topic': s.topic_name,
                    'state': ['ONLINE', 'STALE', 'FAILED'][s.state],
                    'frequency_hz': round(s.frequency_hz, 2),
                    'seconds_since_last_msg': round(s.seconds_since_last_msg, 2),
                    'failure_count': s.failure_count,
                    'criticality': (
                        'critical'
                        if s.criticality == SensorStatus.CRITICAL
                        else 'non_critical'
                    ),
                }
                for s in self._sensor_heartbeats.sensors
            ],
            'active_faults': faults,
        }
        health.dashboard_json = json.dumps(dashboard)
        return health

    def _on_timer(self):
        health = self._build_health()
        self._latest_health = health
        self._health_pub.publish(health)
        self._dashboard_pub.publish(String(data=health.dashboard_json))

        diag_array = DiagnosticArray()
        diag_array.header = health.header
        top_status = DiagnosticStatus()
        top_status.name = 'health_aggregator: robot_overall'
        top_status.hardware_id = 'warehouse_robot'
        level_map = {
            BOOTING: DiagnosticStatus.STALE,
            HEALTHY: DiagnosticStatus.OK,
            DEGRADED: DiagnosticStatus.WARN,
            WARNING: DiagnosticStatus.WARN,
            CRITICAL: DiagnosticStatus.ERROR,
            E_STOP: DiagnosticStatus.ERROR,
        }
        top_status.level = level_map.get(health.health_state, DiagnosticStatus.ERROR)
        top_status.message = (
            '; '.join(health.active_faults) if health.active_faults
            else f'All systems nominal ({state_name(health.health_state)})'
        )
        top_status.values = [
            KeyValue(key='health_state', value=state_name(health.health_state)),
            KeyValue(key='battery_percentage', value=f'{health.battery.percentage:.1f}'),
            KeyValue(key='e_stop_active', value=str(health.e_stop_active)),
            KeyValue(key='safety_controller_state', value=self._safety_state),
        ]
        diag_array.status.append(top_status)
        self._diag_pub.publish(diag_array)

    def _on_get_health(self, request, response):
        response.health = self._latest_health
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HealthAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
