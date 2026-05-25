"""WebSocket bridge server — connects the robot backend to the web frontend.

Provides:
  - Static file serving for the web UI
  - WebSocket endpoint at /ws for real-time data push
  - Command relay from browser → robot system

Usage:
    python -m src.web_server [--host 0.0.0.0] [--port 8080] [--robot-host 127.0.0.1] [--robot-port 9090]
"""

from __future__ import annotations

import asyncio
import json
import mimetypes
import time
from pathlib import Path

from .common.enums import ActionType
from .common.models import Action, Command
from .main import build_system, demo_task_sequence

STATIC_DIR = Path(__file__).parent / "static"


class WebRobotBridge:
    """Bridges WebSocket clients to the robot control system."""

    def __init__(self, system: dict):
        self.system = system
        self._clients: set = set()
        self._task_mgr = system["task_mgr"]
        self._status_mgr = system["status_mgr"]
        self._action_scheduler = system["action_scheduler"]
        self._motion_planner = system["motion_planner"]
        self._log = system["log"]

        # Subscribe to status for push
        self._status_mgr.subscribe_status(self._on_status)

    def _on_status(self, status):
        """Called by StatusManager when robot status changes."""
        # Push will happen in the broadcast loop
        pass

    async def handle_client(self, ws, path):
        """Handle a WebSocket client connection."""
        self._clients.add(ws)
        self._log.info("web", f"Client connected ({len(self._clients)} total)")
        try:
            async for message in ws:
                await self._handle_message(ws, message)
        except Exception:
            pass
        finally:
            self._clients.discard(ws)
            self._log.info("web", f"Client disconnected ({len(self._clients)} total)")

    async def _handle_message(self, ws, raw: str):
        """Process incoming commands from the web UI."""
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            return

        cmd = msg.get("cmd", "")
        data = msg.get("data", {})

        try:
            if cmd == "start_task":
                task_id = data.get("task_id", "")
                if task_id:
                    self._task_mgr.start_task(task_id)
                else:
                    # Create and start a new demo task
                    self._create_demo_task()

            elif cmd == "stop_task":
                task = self._task_mgr.get_current_task()
                if task:
                    self._task_mgr.stop_task(task.task_id)

            elif cmd == "pause_task":
                task = self._task_mgr.get_current_task()
                if task:
                    self._task_mgr.pause_task(task.task_id)

            elif cmd == "resume_task":
                task = self._task_mgr.get_current_task()
                if task:
                    self._task_mgr.resume_task(task.task_id)

            elif cmd == "send_action":
                action_type = ActionType(data.get("action_type", "stop"))
                params = data.get("params", {})
                action = Action(action_type=action_type, params=params)
                self._action_scheduler.schedule_action(action)

            elif cmd == "emergency_stop":
                self._action_scheduler.interrupt_current_action()

            elif cmd == "create_task":
                name = data.get("name", "Web Task")
                seq = data.get("sequence", [])
                actions = self._motion_planner.build_action_sequence(seq)
                task = self._task_mgr.create_task(name=name, actions=actions)
                self._task_mgr.start_task(task.task_id)

        except Exception as e:
            self._log.error("web", f"Command error: {e}")

    async def broadcast_loop(self):
        """Push status, tasks, and logs to all clients at 10Hz."""
        while True:
            await asyncio.sleep(0.1)
            if not self._clients:
                continue

            status = self._status_mgr.get_robot_status()
            current_task = self._task_mgr.get_current_task()
            tasks = self._task_mgr.get_all_tasks()
            logs = self._status_mgr.get_logs(40)
            current_action = self._action_scheduler.get_current_action()

            payload = {
                "type": "state_update",
                "status": status.to_dict(),
                "current_task": current_task.to_dict() if current_task else None,
                "tasks": [t.to_dict() for t in tasks],
                "current_action": current_action.to_dict() if current_action else None,
                "logs": logs,
                "timestamp": time.time(),
            }

            raw = json.dumps(payload, ensure_ascii=False)
            dead = set()
            for ws in self._clients:
                try:
                    await ws.send(raw)
                except Exception:
                    dead.add(ws)
            self._clients -= dead

    def _create_demo_task(self):
        """Create and start a demo 5-action task."""
        actions = self._motion_planner.build_action_sequence([
            {"type": "walk_straight", "distance": 2.0, "speed": 0.5},
            {"type": "turn_in_place", "angle": 90},
            {"type": "walk_straight", "distance": 1.0},
            {"type": "turn_walk", "distance": 1.5, "angle": 45},
            {"type": "stop"},
        ])
        task = self._task_mgr.create_task(
            name="Demo: 5-Action Sequence",
            actions=actions,
        )
        self._task_mgr.start_task(task.task_id)
        return task


def run_web(host: str = "0.0.0.0", port: int = 8080,
            robot_host: str = "127.0.0.1", robot_port: int = 9090):
    """Start the web server with robot backend.

    Serves the web UI via HTTP and handles WebSocket connections on the same port.
    Open http://localhost:8080 in your browser.
    """
    try:
        import websockets
    except ImportError:
        print("[Web] websockets not installed. Run: pip install websockets")
        return

    # Build the robot system
    system = build_system()
    log = system["log"]
    comm = system["comm"]

    log.info("web", f"Starting web server on http://{host}:{port}")

    # Connect to robot/simulator
    if not comm.connect(robot_host, robot_port):
        log.warning("web", f"Could not connect to robot at {robot_host}:{robot_port}")
    else:
        log.info("web", f"Connected to robot at {robot_host}:{robot_port}")

    # Create demo task
    demo_task_sequence(system)

    bridge = WebRobotBridge(system)

    def http_handler(connection, request):
        """Serve static files for HTTP GET, allow WebSocket upgrades."""
        from websockets.http11 import Response, Headers

        # Allow WebSocket upgrade to proceed normally
        if request.headers.get("Upgrade", "").lower() == "websocket":
            return None

        path = request.path or "/"
        if path == "/" or path == "/index.html":
            html_path = STATIC_DIR / "index.html"
            if html_path.exists():
                h = Headers()
                h["Content-Type"] = "text/html; charset=utf-8"
                return Response(200, "OK", h, html_path.read_bytes())
            return Response(404, "Not Found", Headers(), b"Not Found")

        file_path = STATIC_DIR / path.lstrip("/")
        if file_path.exists() and file_path.is_file():
            ct, _ = mimetypes.guess_type(str(file_path))
            h = Headers()
            h["Content-Type"] = ct or "application/octet-stream"
            return Response(200, "OK", h, file_path.read_bytes())

        return Response(404, "Not Found", Headers(), b"Not Found")

    async def serve():
        import websockets
        async with websockets.serve(
            bridge.handle_client, host, port,
            process_request=http_handler,
            max_size=2**20,
            ping_interval=30,
            ping_timeout=10,
        ):
            log.info("web", f"Web dashboard → http://{host}:{port}")
            await bridge.broadcast_loop()

    try:
        asyncio.run(serve())
    except KeyboardInterrupt:
        log.info("web", "Shutting down...")
        comm.disconnect()
