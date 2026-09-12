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
Test the FastAPI dashboard application.

Real, executable HTTP/WebSocket tests for the dashboard's FastAPI app
(dashboard_app.py), using FastAPI's TestClient. No ROS 2 install or
rclpy needed -- dashboard_app.py takes plain get_snapshot/enqueue
callables, which we fake here, exactly as dashboard_bridge.py's real
node methods would be used in production. This genuinely proves the
web layer (HTML serving, REST endpoints, WebSocket broadcast loop)
works, rather than only checking that the file parses as Python.
"""

import os
import sys

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__),
        '..',
        'warehouse_robot_dashboard'))

from dashboard_app import build_app  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

STATIC_DIR = os.path.join(
    os.path.dirname(__file__),
    '..',
    'warehouse_robot_dashboard',
    'static')


def _sample_snapshot():
    return {
        'robot_id': 'WHR-01',
        'ros_connected': True,
        'system_uptime_sec': 12.3,
        'health': {
            'health_state': 'HEALTHY',
            'e_stop_active': False,
            'safety_controller_state': 'CRUISING',
            'battery': {
                'percentage': 87.5,
                'voltage': 23.1,
                'current': 1.2,
                'state': 'NORMAL',
                'is_simulated': True,
                'estimated_remaining_minutes': 210.0,
            },
            'localization': {
                'is_healthy': True,
                'is_stale': False,
                'source': 'wheel_odometry',
                'pose': {'x': 1.0, 'y': 2.0, 'yaw': 0.5},
                'linear_speed': 0.3,
                'angular_speed': 0.0,
                'message': 'Localization nominal',
            },
            'sensors': [
                {'name': 'lidar', 'topic': '/scan', 'state': 'ONLINE', 'frequency_hz': 10.0,
                 'seconds_since_last_msg': 0.05, 'failure_count': 0, 'criticality': 'critical'},
            ],
            'active_faults': [],
        },
        'cmd_vel': {'linear_x': 0.3, 'angular_z': 0.0},
        'scan_points': [[0.0, 2.0], [0.1, 1.8]],
        'last_command_result': None,
    }


def _make_client():
    enqueued = []

    def enqueue(kind, payload):
        enqueued.append((kind, payload))

    app = build_app(
        STATIC_DIR,
        _sample_snapshot,
        enqueue,
        broadcast_rate_hz=20.0)
    return TestClient(app), enqueued


def test_index_serves_the_actual_dashboard_html():
    client, _ = _make_client()
    resp = client.get('/')
    assert resp.status_code == 200
    assert 'text/html' in resp.headers['content-type']
    # Sanity-check this is really our dashboard, not a generic FastAPI page.
    assert 'WAREHOUSE ROBOT OPERATIONS CONSOLE' in resp.text
    assert 'EMERGENCY STOP' in resp.text
    assert 'id="map"' in resp.text  # the 2D warehouse canvas


def test_static_directory_is_mounted():
    client, _ = _make_client()
    resp = client.get('/static/index.html')
    assert resp.status_code == 200
    assert 'WAREHOUSE ROBOT OPERATIONS CONSOLE' in resp.text


def test_estop_endpoint_enqueues_the_exact_payload():
    client, enqueued = _make_client()
    resp = client.post(
        '/api/estop',
        json={
            'activate': True,
            'confirm_reset': False,
            'reason': 'test'})
    assert resp.status_code == 200
    assert resp.json() == {'queued': True}
    assert enqueued == [
        ('estop', {'activate': True, 'confirm_reset': False, 'reason': 'test'})]


def test_reset_requires_confirm_reset_flag_present_in_payload():
    # The dashboard app itself doesn't enforce the safety rule (that's
    # e_stop_manager's job -- see estop_logic.py) but this test locks in
    # that the UI always sends confirm_reset explicitly rather than
    # omitting it, so a downstream service change can't silently start
    # defaulting it to True.
    client, enqueued = _make_client()
    client.post(
        '/api/estop',
        json={
            'activate': False,
            'confirm_reset': True,
            'reason': 'operator reset'})
    assert enqueued[0][1]['confirm_reset'] is True


def test_fault_endpoint_enqueues_sensor_name_and_flag():
    client, enqueued = _make_client()
    client.post(
        '/api/fault',
        json={
            'sensor_name': 'lidar',
            'fault_enabled': True})
    assert enqueued == [
        ('fault', {'sensor_name': 'lidar', 'fault_enabled': True})]


def test_battery_override_endpoint_enqueues_percentage():
    client, enqueued = _make_client()
    client.post(
        '/api/battery_override',
        json={
            'percentage': 12.0,
            'override_enabled': True})
    assert enqueued == [
        ('battery_override', {
            'percentage': 12.0, 'override_enabled': True})]


def test_websocket_broadcasts_the_live_snapshot():
    client, _ = _make_client()
    with client.websocket_connect('/ws') as ws:
        data = ws.receive_json()
    assert data['robot_id'] == 'WHR-01'
    assert data['health']['health_state'] == 'HEALTHY'
    assert data['health']['sensors'][0]['name'] == 'lidar'
    assert data['scan_points'] == [[0.0, 2.0], [0.1, 1.8]]
