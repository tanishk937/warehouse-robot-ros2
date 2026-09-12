#!/usr/bin/env python3
# Copyright 2026 Tanishk Patidar
#
# This source code is provided for viewing, evaluation, educational,
# and portfolio purposes. All rights reserved.
#
# See the repository LICENSE file for terms governing copying,
# modification, distribution, and commercial use.

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from warehouse_robot_control.safety_logic import (
    battery_speed_scale,
    decide_motion,
    is_scan_geometrically_valid,
    sector_min_range,
)
from warehouse_robot_msgs.msg import BatteryStatus


class SafetyController(Node):

    STATE_BOOTING = 'BOOTING'
    STATE_CRUISING = 'CRUISING'
    STATE_SLOW = 'SLOW'
    STATE_OBSTACLE_STOP = 'OBSTACLE_STOP'
    STATE_LOW_BATTERY = 'DEGRADED_LOW_BATTERY'
    STATE_BATTERY_CRITICAL = 'BATTERY_CRITICAL_STOP'
    STATE_BATTERY_STALE = 'BATTERY_STALE'
    STATE_LIDAR_STALE = 'LIDAR_STALE'
    STATE_ODOM_STALE = 'ODOM_STALE'
    STATE_ESTOPPED = 'E_STOPPED'

    def __init__(self):
        super().__init__('safety_controller')

        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('cruise_linear_speed', 0.3)
        self.declare_parameter('avoid_angular_speed', 0.6)
        self.declare_parameter('obstacle_stop_distance', 0.45)
        self.declare_parameter('obstacle_slow_distance', 1.0)
        self.declare_parameter('low_battery_speed_scale', 0.4)
        self.declare_parameter('front_sector_deg', 60.0)
        # --- Critical-fix parameters: never hardcoded, see docs/AUDIT.md #1 ---
        self.declare_parameter('lidar_timeout_sec', 0.5)
        self.declare_parameter('lidar_recovery_consecutive_msgs', 5)
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('battery_timeout_sec', 1.5)

        self._control_rate_hz = self.get_parameter('control_rate_hz').value
        self._cruise_speed = self.get_parameter('cruise_linear_speed').value
        self._avoid_angular_speed = self.get_parameter('avoid_angular_speed').value
        self._stop_distance = self.get_parameter('obstacle_stop_distance').value
        self._slow_distance = self.get_parameter('obstacle_slow_distance').value
        self._low_battery_scale = self.get_parameter('low_battery_speed_scale').value
        self._front_sector_deg = self.get_parameter('front_sector_deg').value
        self._lidar_timeout_sec = self.get_parameter('lidar_timeout_sec').value
        self._lidar_recovery_consecutive_msgs = int(
            self.get_parameter('lidar_recovery_consecutive_msgs').value
        )
        self._odom_timeout_sec = self.get_parameter('odom_timeout_sec').value
        self._battery_timeout_sec = self.get_parameter('battery_timeout_sec').value

        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._latest_scan = None
        self._last_scan_wall_time = None   # independent of message content -- see class docstring
        self._last_odom_wall_time = None
        self._last_battery_wall_time = None
        self._consecutive_fresh_scans = 0
        self._lidar_was_stale = True       # Start stale until proven fresh (fail-safe default).

        self._e_stop_active = False
        self._battery_state = BatteryStatus.NORMAL
        self._last_logged_state = None

        self.create_subscription(LaserScan, '/scan', self._on_scan, best_effort_qos)
        self.create_subscription(Odometry, '/odom', self._on_odom, reliable_qos)
        self.create_subscription(Bool, '/e_stop/status', self._on_e_stop, latched_qos)
        self.create_subscription(BatteryStatus, '/battery/status', self._on_battery, latched_qos)

        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', reliable_qos)
        self._state_pub = self.create_publisher(String, '/safety_controller/state', reliable_qos)

        period = 1.0 / self._control_rate_hz if self._control_rate_hz > 0 else 0.1
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'[SAFETY] safety_controller started '
            f'(lidar_timeout_sec={self._lidar_timeout_sec}, '
            f'recovery_consecutive_msgs={self._lidar_recovery_consecutive_msgs})'
        )

    # ---- subscriptions -----------------------------------------------

    def _on_scan(self, msg: LaserScan):
        self._latest_scan = msg
        self._last_scan_wall_time = time.monotonic()
        if self._lidar_was_stale:
            self._consecutive_fresh_scans += 1

    def _on_odom(self, msg: Odometry):
        self._last_odom_wall_time = time.monotonic()

    def _on_e_stop(self, msg: Bool):
        if msg.data != self._e_stop_active:
            level = 'ACTIVE' if msg.data else 'CLEARED'
            self.get_logger().warn(f'[SAFETY] E-stop observed {level}')
        self._e_stop_active = msg.data

    def _on_battery(self, msg: BatteryStatus):
        self._battery_state = msg.state
        self._last_battery_wall_time = time.monotonic()

    # ---- LiDAR freshness (the critical fix) ---------------------------

    def _lidar_is_fresh(self) -> bool:
        """
        Return whether the latest LiDAR message is fresh.

        Re-evaluate this every control cycle so a LiDAR that silently stops
        publishing is caught without requiring another message.
        """
        if self._last_scan_wall_time is None:
            return False
        age = time.monotonic() - self._last_scan_wall_time
        return age <= self._lidar_timeout_sec

    def _update_lidar_recovery_gate(self, fresh_now: bool) -> bool:
        """
        Apply the debounced LiDAR recovery policy.

        Once LiDAR_STALE has been declared, N consecutive fresh scans are
        required before obstacle avoidance is trusted again.
        """
        if not fresh_now:
            self._consecutive_fresh_scans = 0
            self._lidar_was_stale = True
            return False

        if self._lidar_was_stale:
            if self._consecutive_fresh_scans >= self._lidar_recovery_consecutive_msgs:
                self._lidar_was_stale = False
                self._consecutive_fresh_scans = 0
                self.get_logger().info('[SAFETY] LiDAR recovered — resuming obstacle avoidance')
                return True
            return False

        return True

    # ---- Odometry freshness ---------------------------------------------

    def _odom_is_fresh(self) -> bool:
        """
        Return True only while /odom has been received recently.

        This is checked every control cycle so a silent odometry failure
        cannot leave the last known motion state trusted indefinitely.
        """
        if self._last_odom_wall_time is None:
            return False
        age = time.monotonic() - self._last_odom_wall_time
        return age <= self._odom_timeout_sec

    # ---- control loop ---------------------------------------------------

    def _battery_is_fresh(self) -> bool:
        """
        Return True only while /battery/status has been received recently.

        This is checked every control cycle so a silent battery telemetry
        failure cannot leave the last known battery state trusted indefinitely.
        """
        if self._last_battery_wall_time is None:
            return False
        age = time.monotonic() - self._last_battery_wall_time
        return age <= self._battery_timeout_sec

    def _control_loop(self):
        # Priority 1: E-stop. Nothing else is even evaluated.
        if self._e_stop_active:
            self._publish(0.0, 0.0, self.STATE_ESTOPPED)
            return

        # Priority 2: LiDAR freshness, evaluated independently of whether
        # a scan object exists in memory (docs/AUDIT.md #1).
        fresh_now = self._lidar_is_fresh()
        avoidance_allowed = self._update_lidar_recovery_gate(fresh_now)
        if not avoidance_allowed:
            reason = (
                'no scan ever received'
                if self._last_scan_wall_time is None
                else (
                    f'last scan {time.monotonic() - self._last_scan_wall_time:.2f}s ago '
                    f'(timeout {self._lidar_timeout_sec}s)'
                )
            )
            self._log_state_change(
                self.STATE_LIDAR_STALE,
                f'[SAFETY] LiDAR stale: {reason} — COMMAND STOP',
            )
            self._publish(0.0, 0.0, self.STATE_LIDAR_STALE)
            return

        # Priority 3: odometry freshness.
        # Odometry is required for safe motion; if it silently stops,
        # command a hard stop rather than trusting stale state.
        if not self._odom_is_fresh():
            reason = 'no odometry ever received' if self._last_odom_wall_time is None else \
                f'last odometry {time.monotonic() - self._last_odom_wall_time:.2f}s ago ' \
                f'(timeout {self._odom_timeout_sec}s)'
            self._log_state_change(
                self.STATE_ODOM_STALE,
                f'[SAFETY] Odometry stale: {reason} — COMMAND STOP'
            )
            self._publish(0.0, 0.0, self.STATE_ODOM_STALE)
            return

        # Priority 4: battery telemetry freshness.
        # Battery status is required for safe motion; if it silently stops,
        # command a hard stop rather than trusting stale battery state.
        if not self._battery_is_fresh():
            reason = (
                'no battery status ever received'
                if self._last_battery_wall_time is None
                else (
                    f'last battery status '
                    f'{time.monotonic() - self._last_battery_wall_time:.2f}s ago '
                    f'(timeout {self._battery_timeout_sec}s)'
                )
            )
            self._log_state_change(
                self.STATE_BATTERY_STALE,
                f'[SAFETY] Battery telemetry stale: {reason} — COMMAND STOP'
            )
            self._publish(0.0, 0.0, self.STATE_BATTERY_STALE)
            return

        # Priority 5: critical battery.
        if self._battery_state == BatteryStatus.CRITICAL:
            self._log_state_change(
                self.STATE_BATTERY_CRITICAL,
                '[SAFETY] Battery CRITICAL — COMMAND STOP',
            )
            self._publish(0.0, 0.0, self.STATE_BATTERY_CRITICAL)
            return

        # Priority 6/7: obstacle avoidance, with battery-LOW speed derate folded in.
        scan = self._latest_scan
        if not is_scan_geometrically_valid(len(scan.ranges), scan.angle_increment):
            self._log_state_change(
                self.STATE_LIDAR_STALE,
                ('[SAFETY] LiDAR scan geometrically invalid (empty or zero angle_increment) '
                 '— COMMAND STOP')
            )
            self._publish(0.0, 0.0, self.STATE_LIDAR_STALE)
            return

        half_width = math.radians(self._front_sector_deg / 2.0)
        front_min = sector_min_range(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max, 0.0, half_width,
        )
        left_min = sector_min_range(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max, math.radians(60.0), half_width,
        )
        right_min = sector_min_range(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max, math.radians(-60.0), half_width,
        )

        speed_scale = battery_speed_scale(
            self._battery_state, BatteryStatus.NORMAL, BatteryStatus.LOW,
            BatteryStatus.CRITICAL, self._low_battery_scale,
        )
        base_state = (
            self.STATE_LOW_BATTERY
            if self._battery_state == BatteryStatus.LOW
            else self.STATE_CRUISING
        )

        decision = decide_motion(
            front_min, left_min, right_min,
            self._stop_distance, self._slow_distance,
            self._cruise_speed, self._avoid_angular_speed,
            speed_scale, base_state,
        )
        self._log_state_change(decision.state, f'[SAFETY] State -> {decision.state}')
        self._publish(decision.linear_x, decision.angular_z, decision.state)

    def _publish(self, linear_x, angular_z, state):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self._cmd_vel_pub.publish(cmd)
        self._state_pub.publish(String(data=state))

    def _log_state_change(self, state, message):
        if state != self._last_logged_state:
            self.get_logger().warn(message) if state in (
                self.STATE_LIDAR_STALE, self.STATE_BATTERY_CRITICAL, self.STATE_ESTOPPED,
            ) else self.get_logger().info(message)
            self._last_logged_state = state


def main(args=None):
    rclpy.init(args=args)
    node = SafetyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
