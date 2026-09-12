#!/usr/bin/env python3
# Copyright 2026 Tanishk Patidar
#
# This source code is provided for viewing, evaluation, educational,
# and portfolio purposes. All rights reserved.
#
# See the repository LICENSE file for terms governing copying,
# modification, distribution, and commercial use.


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
