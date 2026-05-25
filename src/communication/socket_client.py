"""SocketClient — Member C

Low-level TCP/WebSocket connection to the robot controller.
Handles connect/disconnect/reconnect and provides send/receive primitives.

Supports:
  - TCP socket (for simulation and wired connection)
  - WebSocket (for wireless/remote connection)
  - Automatic reconnection with exponential backoff
"""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Callable, Optional


class SocketClient:
    """TCP socket client for robot communication.

    Manages the connection lifecycle and provides raw send/receive.
    Thread-safe for sending; receiving runs in a background thread.
    """

    RECONNECT_BASE_DELAY = 1.0    # seconds
    RECONNECT_MAX_DELAY = 30.0    # seconds
    RECEIVE_BUFFER = 4096

    def __init__(self):
        self._sock: Optional[socket.socket] = None
        self._host: str = ""
        self._port: int = 0
        self._connected = False
        self._running = False
        self._lock = threading.Lock()
        self._recv_thread: Optional[threading.Thread] = None

        # Callbacks
        self._on_data: Optional[Callable[[bytes], None]] = None
        self._on_connected: Optional[Callable[[], None]] = None
        self._on_disconnected: Optional[Callable[[str], None]] = None

    # ── Connection ──────────────────────────────────────────────

    def connect(self, host: str, port: int, timeout: float = 5.0) -> bool:
        """Establish TCP connection to the robot controller."""
        with self._lock:
            if self._connected:
                return True
            self._host = host
            self._port = port

        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(timeout)
            self._sock.connect((host, port))
            self._sock.settimeout(None)  # back to blocking for recv
        except (socket.timeout, ConnectionRefusedError, OSError) as e:
            self._sock = None
            return False

        with self._lock:
            self._connected = True
            self._running = True

        if self._on_connected:
            self._on_connected()

        # Start receive loop
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

        return True

    def disconnect(self) -> None:
        """Close the connection gracefully."""
        with self._lock:
            self._running = False
            self._connected = False

        if self._sock:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
            self._sock = None

        if self._on_disconnected:
            self._on_disconnected("manual")

    # ── Send ────────────────────────────────────────────────────

    def send(self, data: bytes) -> bool:
        """Send raw bytes to the robot. Thread-safe."""
        sock = self._sock
        if sock is None:
            return False
        try:
            # Prefix with length for framing
            length = len(data).to_bytes(4, "big")
            sock.sendall(length + data)
            return True
        except (BrokenPipeError, ConnectionResetError, OSError):
            self._handle_disconnect("send_error")
            return False

    def send_json(self, obj: dict) -> bool:
        """Serialize dict to JSON and send."""
        try:
            return self.send(json.dumps(obj).encode("utf-8"))
        except Exception:
            return False

    # ── Receive callbacks ───────────────────────────────────────

    def on_data(self, callback: Callable[[bytes], None]) -> None:
        self._on_data = callback

    def on_connected(self, callback: Callable[[], None]) -> None:
        self._on_connected = callback

    def on_disconnected(self, callback: Callable[[str], None]) -> None:
        self._on_disconnected = callback

    # ── Status ──────────────────────────────────────────────────

    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    # ── Internal ────────────────────────────────────────────────

    def _recv_loop(self) -> None:
        """Background receive loop. Reads framed messages."""
        buffer = b""
        while self._running:
            try:
                sock = self._sock
                if sock is None:
                    break
                data = sock.recv(self.RECEIVE_BUFFER)
                if not data:
                    # Connection closed by remote
                    self._handle_disconnect("remote_close")
                    break

                buffer += data
                # Parse framed messages
                while len(buffer) >= 4:
                    msg_len = int.from_bytes(buffer[:4], "big")
                    if len(buffer) < 4 + msg_len:
                        break
                    msg = buffer[4:4 + msg_len]
                    buffer = buffer[4 + msg_len:]
                    if self._on_data:
                        self._on_data(msg)

            except (socket.timeout, BlockingIOError):
                continue
            except (ConnectionResetError, BrokenPipeError, OSError):
                self._handle_disconnect("recv_error")
                break

    def _handle_disconnect(self, reason: str) -> None:
        with self._lock:
            was_connected = self._connected
            self._connected = False
            self._running = False
        self._sock = None
        if was_connected and self._on_disconnected:
            self._on_disconnected(reason)
        # Attempt reconnection
        if was_connected:
            self._attempt_reconnect()

    def _attempt_reconnect(self) -> None:
        """Try to reconnect with exponential backoff."""
        delay = self.RECONNECT_BASE_DELAY
        while True:
            time.sleep(delay)
            if self.connect(self._host, self._port):
                break
            delay = min(delay * 2, self.RECONNECT_MAX_DELAY)
