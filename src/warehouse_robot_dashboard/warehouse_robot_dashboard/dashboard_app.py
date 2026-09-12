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
Provide the FastAPI web application for the warehouse robot dashboard.

The FastAPI application itself is deliberately kept free of any rclpy
import so it can be exercised with FastAPI's TestClient in plain
pytest (see test/test_dashboard_app.py) without a ROS 2 install.
dashboard_bridge.py is the thin ROS glue that supplies the two
callables this module needs:

    get_snapshot() -> dict      : the latest cached state to broadcast
    enqueue(kind, payload)      : hand a requested command off to the
                                   node's own rclpy thread for execution

This mirrors the separation already used for safety_logic.py,
estop_logic.py and health_state_logic.py in warehouse_robot_control:
pure/testable logic in one module, ROS wiring in another.
"""

import asyncio
import os

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

STATIC_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    'static',
)

def build_app(static_dir: str, get_snapshot, enqueue,
              broadcast_rate_hz: float = 5.0) -> FastAPI:
    app = FastAPI(title='Warehouse Robot Dashboard Bridge')
    index_path = os.path.join(static_dir, 'index.html')

    @app.get('/', response_class=HTMLResponse)
    async def index():
        with open(index_path, 'r') as f:
            return f.read()

    if os.path.isdir(static_dir):
        app.mount('/static', StaticFiles(directory=static_dir), name='static')

    @app.websocket('/ws')
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()
        period = 1.0 / broadcast_rate_hz if broadcast_rate_hz > 0 else 0.2
        try:
            while True:
                await websocket.send_json(get_snapshot())
                await asyncio.sleep(period)
        except WebSocketDisconnect:
            pass

    @app.post('/api/estop')
    async def api_estop(payload: dict):
        enqueue('estop', payload)
        return {'queued': True}

    @app.post('/api/fault')
    async def api_fault(payload: dict):
        enqueue('fault', payload)
        return {'queued': True}

    @app.post('/api/battery_override')
    async def api_battery_override(payload: dict):
        enqueue('battery_override', payload)
        return {'queued': True}

    return app
