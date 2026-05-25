#!/usr/bin/env python3
"""Simulation runner — starts a mock robot server for testing.

Usage:
    python scripts/run_simulation.py           # server + headless client
    python scripts/run_simulation.py --ui      # server + UI client
    python scripts/run_simulation.py --server-only  # server only (connect with another process)
"""

import argparse
import json
import socket
import struct
import threading
import time
import sys
from pathlib import Path

# Add project root to path so `from src.xxx import ...` works
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


class MockRobotServer:
    """A simple mock robot server for development/testing.

    Listens on TCP port, echoes status data, and simulates action execution.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 9090):
        self._host = host
        self._port = port
        self._running = False
        self._position = [0.0, 0.0, 0.0]
        self._orientation = [0.0, 0.0, 0.0]
        self._battery = 100.0

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._host, self._port))
        self._sock.listen(1)
        self._sock.settimeout(1.0)
        self._running = True

        print(f"[MockRobot] Listening on {self._host}:{self._port}")

        while self._running:
            try:
                conn, addr = self._sock.accept()
                print(f"[MockRobot] Client connected from {addr}")
                t = threading.Thread(target=self._handle_client, args=(conn,), daemon=True)
                t.start()
            except socket.timeout:
                continue

    def stop(self):
        self._running = False
        if self._sock:
            self._sock.close()

    def _handle_client(self, conn: socket.socket):
        conn.settimeout(0.5)
        # Send initial status
        self._send_msg(conn, self._make_status_msg())

        # Start periodic status updates
        def send_status_loop():
            while self._running:
                time.sleep(0.5)
                try:
                    self._send_msg(conn, self._make_status_msg())
                except Exception:
                    break

        status_thread = threading.Thread(target=send_status_loop, daemon=True)
        status_thread.start()

        buffer = b""
        while self._running:
            try:
                data = conn.recv(4096)
                if not data:
                    break
                buffer += data
                while len(buffer) >= 4:
                    msg_len = struct.unpack(">I", buffer[:4])[0]
                    if len(buffer) < 4 + msg_len:
                        break
                    msg = buffer[4:4 + msg_len]
                    buffer = buffer[4 + msg_len:]
                    self._handle_command(conn, msg)
            except socket.timeout:
                continue
            except Exception:
                break

        print("[MockRobot] Client disconnected")

    def _handle_command(self, conn: socket.socket, raw: bytes):
        try:
            msg = json.loads(raw.decode("utf-8"))
        except Exception:
            return

        msg_type = msg.get("type", "")

        # Handle heartbeat — echo back immediately
        if msg_type == "heartbeat":
            self._send_msg(conn, {
                "type": "heartbeat",
                "seq": msg.get("seq", 0),
                "timestamp": time.time(),
            })
            return

        # Handle action command
        action_type = msg.get("action_type", "")
        action_id = msg.get("command_id", "")
        print(f"[MockRobot] Received: {action_type} (id={action_id})")

        # Simulate action execution delay
        time.sleep(0.3)

        # Send action complete
        self._send_msg(conn, {
            "type": "action_complete",
            "action_id": action_id,
            "success": True,
            "error": "",
        })

        # Update mock position based on action
        params = msg.get("params", {})
        if action_type == "walk_straight":
            distance = params.get("distance_m", 0)
            self._position[1] += distance  # move forward in Y
        elif action_type == "turn_in_place":
            angle = params.get("angle_deg", 0)
            self._orientation[2] += angle
        elif action_type == "stop":
            pass

    def _make_status_msg(self):
        self._battery -= 0.01  # simulate battery drain
        return {
            "type": "status",
            "data": {
                "state": "idle",
                "battery": max(0, self._battery),
                "position": list(self._position),
                "orientation": list(self._orientation),
                "velocity": 0.0,
                "current_action_id": "",
                "error_code": 0,
                "timestamp": time.time(),
            },
        }

    @staticmethod
    def _send_msg(conn: socket.socket, obj: dict):
        data = json.dumps(obj).encode("utf-8")
        conn.sendall(struct.pack(">I", len(data)) + data)


def main():
    parser = argparse.ArgumentParser(description="Robot simulation runner")
    parser.add_argument("--server-only", action="store_true", help="Run only the mock server")
    parser.add_argument("--ui", action="store_true", help="Run client with PyQt6 UI")
    parser.add_argument("--web", action="store_true", help="Run client with web dashboard")
    parser.add_argument("--web-port", type=int, default=8080, help="Web dashboard port (default: 8080)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9090)
    args = parser.parse_args()

    server = MockRobotServer(args.host, args.port)
    server_thread = threading.Thread(target=server.start, daemon=True)
    server_thread.start()
    time.sleep(0.5)

    if args.server_only:
        print("Mock server running. Press Ctrl+C to stop.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            server.stop()
        return

    # Run the actual system
    from src.main import run_headless, run_with_ui, run_with_web

    try:
        if args.ui:
            run_with_ui(args.host, args.port)
        elif args.web:
            run_with_web("0.0.0.0", args.web_port, args.host, args.port)
        else:
            run_headless(args.host, args.port)
    finally:
        server.stop()


if __name__ == "__main__":
    main()
