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
Wheel-odometry localization health monitor.

SCOPE (see docs/AUDIT.md #4 -- stated honestly, not oversold): this
node monitors raw wheel odometry (nav_msgs/Odometry from Gazebo's
diff-drive plugin, published as /odom after fault_injector's
passthrough). It is NOT a fused localization system -- there is no
AMCL and no EKF/UKF in this project. The 'source' field on every
published LocalizationStatus always says exactly what is being
watched, and this message type is intentionally source-agnostic: a
future EKF-fused or AMCL-based estimator could publish the same
message on the same topic and nothing downstream (health_aggregator,
dashboard) would need to change.

Beyond staleness and covariance, this node also flags implausible pose
jumps between consecutive messages (an excessive teleport in one
timestep is treated as a localization fault, since real wheel odometry
never teleports).

Publishes:
    /localization/status   (warehouse_robot_msgs/LocalizationStatus)
Subscribes:
    /odom                  (nav_msgs/Odometry)
"""

import math


from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from warehouse_robot_msgs.msg import LocalizationStatus


class LocalizationMonitor(Node):

    def __init__(self):
        super().__init__('localization_monitor')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('stale_timeout_sec', 1.0)
        self.declare_parameter('covariance_warn_threshold', 0.5)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('max_pose_jump_m', 0.5)
        self.declare_parameter('localization_source', 'wheel_odometry')

        odom_topic = self.get_parameter('odom_topic').value
        self._stale_timeout = self.get_parameter('stale_timeout_sec').value
        self._covariance_threshold = self.get_parameter('covariance_warn_threshold').value
        self._max_pose_jump = self.get_parameter('max_pose_jump_m').value
        self._source = self.get_parameter('localization_source').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value

        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._latest_odom = None
        self._last_odom_stamp_sec = None
        self._last_position = None

        self.create_subscription(Odometry, odom_topic, self._on_odom, reliable_qos)
        self._status_pub = self.create_publisher(
            LocalizationStatus, '/localization/status', reliable_qos
        )

        period = 1.0 / publish_rate_hz if publish_rate_hz > 0 else 0.2
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"[LOCALIZATION] Monitoring '{odom_topic}' (source='{self._source}')"
        )

    def _on_odom(self, msg: Odometry):
        self._latest_odom = msg
        self._last_odom_stamp_sec = self.get_clock().now().nanoseconds / 1e9

    def _on_timer(self):
        status = LocalizationStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = 'odom'
        status.source = self._source

        if self._latest_odom is None:
            status.is_healthy = False
            status.is_stale = True
            status.status_message = 'No odometry received yet'
            self._status_pub.publish(status)
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        age = (
            now_sec - self._last_odom_stamp_sec
            if self._last_odom_stamp_sec is not None
            else float('inf')
        )
        is_stale = age > self._stale_timeout

        odom = self._latest_odom
        position = odom.pose.pose.position
        status.pose = odom.pose.pose
        status.linear_speed = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)
        status.angular_speed = odom.twist.twist.angular.z
        status.is_stale = is_stale

        jump_distance = 0.0
        if self._last_position is not None and not is_stale:
            jump_distance = math.hypot(
                position.x - self._last_position[0], position.y - self._last_position[1]
            )
        self._last_position = (position.x, position.y)
        status.pose_jump_distance = jump_distance
        status.pose_jump_detected = jump_distance > self._max_pose_jump

        cov = odom.pose.covariance
        x_var = cov[0]
        y_var = cov[7]
        yaw_var = cov[35]
        max_var = max(x_var, y_var, yaw_var)
        covariance_ok = max_var < self._covariance_threshold

        status.is_healthy = (not is_stale) and covariance_ok and (not status.pose_jump_detected)

        if is_stale:
            status.status_message = f'Odometry stale ({age:.2f}s since last message)'
        elif status.pose_jump_detected:
            status.status_message = (
                f'Implausible pose jump: {jump_distance:.2f}m in one update '
                f'(limit {self._max_pose_jump}m)'
            )
        elif not covariance_ok:
            status.status_message = f'Pose covariance too high (max={max_var:.3f})'
        else:
            status.status_message = f'Localization nominal (source={self._source})'

        self._status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
