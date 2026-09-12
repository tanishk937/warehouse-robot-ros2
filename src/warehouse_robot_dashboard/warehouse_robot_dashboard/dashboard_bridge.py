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
import queue
import threading
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import uvicorn

from warehouse_robot_dashboard.dashboard_app import build_app, STATIC_DIR
from warehouse_robot_msgs.srv import (
    SetBatteryOverride,
    SetEStop,
    SetSensorFault,
)


class DashboardBridge(Node):

    def __init__(self):
        super().__init__('dashboard_bridge')

        self.declare_parameter('http_host', '0.0.0.0')
        self.declare_parameter('http_port', 8080)
        self.declare_parameter('broadcast_rate_hz', 5.0)
        self.declare_parameter('scan_sample_count', 72)
        self.declare_parameter('robot_id', 'WHR-01')

        self._http_host = self.get_parameter('http_host').value
        self._http_port = int(self.get_parameter('http_port').value)
        self._broadcast_rate_hz = self.get_parameter('broadcast_rate_hz').value
        self._scan_sample_count = int(
            self.get_parameter('scan_sample_count').value)
        self._robot_id = self.get_parameter('robot_id').value

        self._lock = threading.Lock()
        self._start_time = time.monotonic()
        self._last_health_monotonic = None
        self._state = {
            'robot_id': self._robot_id,
            'ros_connected': False,
            'system_uptime_sec': 0.0,
            'health': None,          # raw dict decoded from /robot_health/dashboard_json
            'cmd_vel': {'linear_x': 0.0, 'angular_z': 0.0},
            'scan_points': [],       # [[angle_rad, range_m], ...] sampled, robot-frame
            'last_command_result': None,
        }

        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10,
        )

        self.create_subscription(
            String,
            '/robot_health/dashboard_json',
            self._on_health_json,
            reliable_qos)
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self._on_cmd_vel,
            reliable_qos)
        self.create_subscription(
            LaserScan,
            '/scan',
            self._on_scan,
            best_effort_qos)

        self._estop_client = self.create_client(SetEStop, '/e_stop/set')
        self._fault_client = self.create_client(
            SetSensorFault, '/fault/set_sensor_fault')
        self._battery_override_client = self.create_client(
            SetBatteryOverride, '/battery/set_override')

        self._command_queue = queue.Queue()
        # 20 Hz, keeps UI-perceived latency low
        self.create_timer(0.05, self._drain_command_queue)

        self._app = self._build_app()
        self._server_thread = threading.Thread(
            target=self._run_http_server, daemon=True)
        self._server_thread.start()

        self.get_logger().info(
            f'[DASHBOARD] Serving on http://{self._http_host}:{self._http_port}  '
            f'(WebSocket: ws://{self._http_host}:{self._http_port}/ws)')

    # ---- ROS callbacks -------------------------------------------------

    def _on_health_json(self, msg: String):
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._state['health'] = parsed
            self._last_health_monotonic = time.monotonic()

    def _on_cmd_vel(self, msg: Twist):
        with self._lock:
            self._state['cmd_vel'] = {
                'linear_x': msg.linear.x,
                'angular_z': msg.angular.z}

    def _on_scan(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return
        step = max(1, n // self._scan_sample_count)
        points = []
        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            if i % step == 0 and math.isfinite(
                    r) and msg.range_min <= r <= msg.range_max:
                points.append([round(angle, 4), round(r, 3)])
            angle += msg.angle_increment
        with self._lock:
            self._state['scan_points'] = points

    # ---- outgoing command queue (executed on the rclpy thread) --------

    def _drain_command_queue(self):
        try:
            while True:
                kind, payload = self._command_queue.get_nowait()
                self._dispatch_command(kind, payload)
        except queue.Empty:
            pass

    def _dispatch_command(self, kind, payload):
        if kind == 'estop':
            if not self._estop_client.service_is_ready():
                self._record_result(False, '/e_stop/set service not available')
                return
            req = SetEStop.Request()
            req.activate = bool(payload.get('activate', False))
            req.confirm_reset = bool(payload.get('confirm_reset', False))
            req.reason = str(payload.get('reason', 'dashboard'))
            future = self._estop_client.call_async(req)
            future.add_done_callback(lambda f: self._on_estop_result(f))
        elif kind == 'fault':
            if not self._fault_client.service_is_ready():
                self._record_result(
                    False, '/fault/set_sensor_fault service not available')
                return
            req = SetSensorFault.Request()
            req.sensor_name = str(payload.get('sensor_name', ''))
            req.fault_enabled = bool(payload.get('fault_enabled', False))
            future = self._fault_client.call_async(req)
            future.add_done_callback(
                lambda f: self._on_generic_result(
                    f, 'fault'))
        elif kind == 'battery_override':
            if not self._battery_override_client.service_is_ready():
                self._record_result(
                    False, '/battery/set_override service not available')
                return
            req = SetBatteryOverride.Request()
            req.percentage = float(payload.get('percentage', 100.0))
            req.override_enabled = bool(payload.get('override_enabled', False))
            future = self._battery_override_client.call_async(req)
            future.add_done_callback(
                lambda f: self._on_generic_result(
                    f, 'battery_override'))

    def _on_estop_result(self, future):
        try:
            resp = future.result()
            self._record_result(resp.success, resp.message)
        except Exception as exc:  # noqa: BLE001
            # Surface RPC failures to the UI for operator visibility.

            self._record_result(False, f'E-stop service call failed: {exc}')

    def _on_generic_result(self, future, label):
        try:
            resp = future.result()
            self._record_result(resp.success, f'[{label}] {resp.message}')
        except Exception as exc:  # noqa: BLE001
            self._record_result(False, f'[{label}] service call failed: {exc}')

    def _record_result(self, success, message):

        with self._lock:

            self._state['last_command_result'] = {

                'success': success, 'message': message, 'timestamp': time.time(), }

        if success:
            self.get_logger().info(f'[DASHBOARD] {message}')
        else:
            self.get_logger().warning(f'[DASHBOARD] {message}')
    # ---- snapshot for the websocket loop --------------------------------

    def snapshot(self):
        with self._lock:
            state = dict(self._state)
            last_health = self._last_health_monotonic

        state['system_uptime_sec'] = round(
            time.monotonic() - self._start_time, 1)

        # Dashboard ROS connection watchdog.
        # The dashboard is considered connected only while the ROS
        # health stream has been received within the timeout.
        health_timeout_sec = 2.0
        state['ros_connected'] = (
            last_health is not None
            and (time.monotonic() - last_health) <= health_timeout_sec
        )

        return state

    def enqueue(self, kind, payload):
        self._command_queue.put((kind, payload))

    # ---- FastAPI app -----------------------------------------------------

    def _build_app(self):
        return build_app(
            STATIC_DIR,
            self.snapshot,
            self.enqueue,
            self._broadcast_rate_hz)

    def _run_http_server(self):
        config = uvicorn.Config(
            self._app,
            host=self._http_host,
            port=self._http_port,
            log_level='warning')
        server = uvicorn.Server(config)
        server.run()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
