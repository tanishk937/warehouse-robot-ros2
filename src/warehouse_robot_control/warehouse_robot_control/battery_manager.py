#!/usr/bin/env python3
# Copyright 2026 Tanishk Patidar
#
# This source code is provided for viewing, evaluation, educational,
# and portfolio purposes. All rights reserved.
#
# See the repository LICENSE file for terms governing copying,
# modification, distribution, and commercial use.


from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from warehouse_robot_msgs.msg import BatteryStatus
from warehouse_robot_msgs.srv import SetBatteryOverride


class BatteryManager(Node):

    def __init__(self):
        super().__init__('battery_manager')

        self.declare_parameter('simulate_battery', True)
        self.declare_parameter('update_rate_hz', 2.0)
        self.declare_parameter('nominal_voltage', 24.0)
        self.declare_parameter('min_voltage', 20.0)
        self.declare_parameter('design_capacity_ah', 20.0)
        self.declare_parameter('idle_drain_percent_per_sec', 0.0015)
        self.declare_parameter('moving_drain_multiplier', 6.0)
        self.declare_parameter('low_battery_threshold', 30.0)
        self.declare_parameter('critical_battery_threshold', 15.0)
        self.declare_parameter('idle_current_amps', 1.2)
        self.declare_parameter('moving_current_amps', 5.5)

        self._simulate = self.get_parameter('simulate_battery').value
        self._nominal_voltage = self.get_parameter('nominal_voltage').value
        self._min_voltage = self.get_parameter('min_voltage').value
        self._design_capacity_ah = self.get_parameter('design_capacity_ah').value
        self._idle_drain = self.get_parameter('idle_drain_percent_per_sec').value
        self._moving_multiplier = self.get_parameter('moving_drain_multiplier').value
        self._low_threshold = self.get_parameter('low_battery_threshold').value
        self._critical_threshold = self.get_parameter('critical_battery_threshold').value
        self._idle_current = self.get_parameter('idle_current_amps').value
        self._moving_current = self.get_parameter('moving_current_amps').value
        update_rate_hz = self.get_parameter('update_rate_hz').value

        self._percentage = 100.0
        self._is_moving = False
        self._last_low_flag = None
        self._override_active = False

        reliable_volatile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_transient_local = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        if self._simulate:
            self._battery_state_pub = self.create_publisher(
                BatteryState, '/battery_state', reliable_volatile
            )
        else:
            self.create_subscription(
                BatteryState, '/battery_state', self._on_battery_state, reliable_volatile
            )
            self._battery_state_pub = None

        self._status_pub = self.create_publisher(
            BatteryStatus, '/battery/status', reliable_transient_local
        )
        self._low_battery_pub = self.create_publisher(
            Bool, '/battery/low_battery_trigger', reliable_transient_local
        )

        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_service(SetBatteryOverride, '/battery/set_override', self._on_set_override)

        self._period = 1.0 / update_rate_hz if update_rate_hz > 0 else 0.5
        self._timer = self.create_timer(self._period, self._on_timer)

        self.get_logger().info(
            f'[BATTERY] battery_manager started (simulate_battery={self._simulate}, '
            f'low<{self._low_threshold}%, critical<{self._critical_threshold}%)'
        )

    def _on_cmd_vel(self, msg: Twist):
        speed = (msg.linear.x ** 2 + msg.linear.y ** 2) ** 0.5 + abs(msg.angular.z)
        self._is_moving = speed > 0.01

    def _on_battery_state(self, msg: BatteryState):
        if self._override_active:
            return
        percentage = msg.percentage
        if percentage is not None and percentage == percentage:  # filters out NaN
            if percentage <= 1.0:
                percentage *= 100.0
            self._percentage = max(0.0, min(100.0, percentage))

    def _on_set_override(
        self, request: SetBatteryOverride.Request, response: SetBatteryOverride.Response
    ):
        self._override_active = request.override_enabled
        if request.override_enabled:
            self._percentage = max(0.0, min(100.0, request.percentage))
            self.get_logger().warn(
                f'[BATTERY] Fault-injection override ENABLED: forcing {self._percentage:.1f}%'
            )
            response.message = f'Battery override active at {self._percentage:.1f}%'
        else:
            self.get_logger().warn(
                f'[BATTERY] Fault-injection override CLEARED: resuming simulated '
                f'drain from {self._percentage:.1f}%'
            )
            response.message = 'Battery override cleared; resuming normal behavior'
        response.success = True
        return response

    def _simulate_step(self):
        if self._override_active:
            return
        drain = self._idle_drain * (self._moving_multiplier if self._is_moving else 1.0)
        self._percentage = max(0.0, self._percentage - drain * self._period)

    def _voltage_from_percentage(self):
        fraction = self._percentage / 100.0
        return self._min_voltage + fraction * (self._nominal_voltage - self._min_voltage)

    def _current_draw(self):
        return self._moving_current if self._is_moving else self._idle_current

    def _estimated_remaining_minutes(self):
        if self._percentage <= 0.0:
            return 0.0
        current = max(self._current_draw(), 0.01)
        remaining_ah = self._design_capacity_ah * (self._percentage / 100.0)
        return (remaining_ah / current) * 60.0

    def _on_timer(self):
        voltage = self._voltage_from_percentage()
        current = self._current_draw()

        if self._simulate:
            self._simulate_step()
            voltage = self._voltage_from_percentage()

            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            battery_msg.header.frame_id = 'base_link'
            battery_msg.voltage = float(voltage)
            battery_msg.current = -float(current)  # Negative means discharging.
            battery_msg.percentage = float(self._percentage / 100.0)
            battery_msg.design_capacity = float(self._design_capacity_ah)
            battery_msg.charge = float(self._design_capacity_ah * self._percentage / 100.0)
            battery_msg.present = True
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            battery_msg.power_supply_health = (
                BatteryState.POWER_SUPPLY_HEALTH_GOOD
                if self._percentage > self._critical_threshold
                else BatteryState.POWER_SUPPLY_HEALTH_DEAD
            )
            battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
            self._battery_state_pub.publish(battery_msg)

        status = BatteryStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = 'base_link'
        status.percentage = float(self._percentage)
        status.voltage = float(voltage)
        status.current = float(current)
        status.is_charging = False
        status.is_simulated = bool(self._simulate)
        status.estimated_remaining_minutes = float(self._estimated_remaining_minutes())

        if self._percentage <= self._critical_threshold:
            status.state = BatteryStatus.CRITICAL
        elif self._percentage <= self._low_threshold:
            status.state = BatteryStatus.LOW
        else:
            status.state = BatteryStatus.NORMAL

        self._status_pub.publish(status)

        is_low = status.state != BatteryStatus.NORMAL
        if is_low != self._last_low_flag:
            self._low_battery_pub.publish(Bool(data=is_low))
            self._last_low_flag = is_low
            if is_low:
                self.get_logger().warn(
                    f'[BATTERY] Low-battery trigger ACTIVE at {self._percentage:.1f}% '
                    f'(state={status.state})'
                )
            else:
                self.get_logger().info('[BATTERY] Low-battery trigger cleared')


def main(args=None):
    rclpy.init(args=args)
    node = BatteryManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
