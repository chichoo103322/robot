"""APIService — Member C

High-level communication API that integrates SocketClient, CommandSender,
and HeartbeatManager. This is the main entry point for other modules to
use the communication layer.

Implements ICommunication interface.
"""

from __future__ import annotations

import json
import threading
from typing import Callable, Optional

from ..common.enums import ActionType, RobotState
from ..common.interfaces import ICommunication
from ..common.models import Command, RobotStatus, SensorData
from .socket_client import SocketClient
from .command_sender import CommandSender
from .heartbeat_manager import HeartbeatManager


class APIService(ICommunication):
    """Main communication service — implements ICommunication.

    Wires together:
      - SocketClient (connection)
      - CommandSender (outgoing)
      - HeartbeatManager (health)

    Parses incoming data and dispatches to:
      - StatusManager (via on_status_received callback)
      - ObstacleDetector (via on_sensor_data callback)
    """

    def __init__(self):
        self._socket = SocketClient()
        self._sender = CommandSender(self._socket)
        self._heartbeat = HeartbeatManager()

        # Wire heartbeat to use socket
        self._heartbeat.set_send_func(self._socket.send_json)

        # Callbacks for incoming data
        self._on_status_cb: Optional[Callable[[RobotStatus], None]] = None
        self._on_sensor_cb: Optional[Callable[[SensorData], None]] = None

        # Set up incoming data dispatch
        self._socket.on_data(self._handle_incoming_data)
        self._socket.on_disconnected(self._on_disconnect)

    # ── Connection ──────────────────────────────────────────────

    def connect(self, host: str, port: int) -> bool:
        success = self._socket.connect(host, port)
        if success:
            self._sender.start()
            self._heartbeat.start()
        return success

    def disconnect(self) -> None:
        self._heartbeat.stop()
        self._sender.stop()
        self._socket.disconnect()

    def is_connected(self) -> bool:
        return self._socket.is_connected()

    # ── Sending ─────────────────────────────────────────────────

    def send_command(self, command: Command) -> bool:
        return self._sender.send_command(command)

    def send_command_immediate(self, command: Command) -> bool:
        return self._sender.send_immediate(command)

    # ── Receiving ───────────────────────────────────────────────

    def start_receiving(self) -> None:
        """Already started in connect(). Explicit call for interface compliance."""
        pass

    def on_status_received(self, callback: Callable[[RobotStatus], None]) -> None:
        self._on_status_cb = callback

    def on_sensor_data(self, callback: Callable[[SensorData], None]) -> None:
        self._on_sensor_cb = callback

    # ── Heartbeat ───────────────────────────────────────────────

    def start_heartbeat(self, interval_s: float = 1.0) -> None:
        self._heartbeat.start(interval_s)

    def on_heartbeat_timeout(self, callback: Callable[[], None]) -> None:
        self._heartbeat.on_timeout(callback)

    def on_heartbeat_latency(self, callback: Callable[[float], None]) -> None:
        self._heartbeat.on_beat(callback)

    def is_heartbeat_alive(self) -> bool:
        return self._heartbeat.is_alive()

    # ── Internal ────────────────────────────────────────────────

    def _handle_incoming_data(self, data: bytes) -> None:
        """Parse incoming data and dispatch to appropriate handler."""
        try:
            msg = json.loads(data.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return

        msg_type = msg.get("type", "")

        if msg_type == "status":
            # Robot status update
            status = RobotStatus.from_dict(msg.get("data", {}))
            if self._on_status_cb:
                self._on_status_cb(status)

        elif msg_type == "sensor":
            # Sensor/vision data
            sensor = SensorData.from_dict(msg.get("data", {}))
            if self._on_sensor_cb:
                self._on_sensor_cb(sensor)

        elif msg_type == "heartbeat":
            # Heartbeat response
            self._heartbeat.on_pong(msg.get("seq", 0))

        elif msg_type == "action_complete":
            # Action execution result from robot
            action_id = msg.get("action_id", "")
            success = msg.get("success", True)
            error = msg.get("error", "")
            # The ActionScheduler should register as a listener
            if hasattr(self, "_on_action_complete_cb") and self._on_action_complete_cb:
                self._on_action_complete_cb(action_id, success, error)

        elif msg_type == "error":
            # Robot error report
            if self._on_status_cb:
                error_status = RobotStatus(
                    state=RobotState.ERROR,
                    error_code=msg.get("code", -1),
                )
                self._on_status_cb(error_status)

    def _on_disconnect(self, reason: str) -> None:
        self._heartbeat.stop()
        if self._on_status_cb:
            self._on_status_cb(RobotStatus(state=RobotState.ERROR, error_code=999))
