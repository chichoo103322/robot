"""HeartbeatManager — Member C

Periodic heartbeat mechanism to monitor robot connection health.
Sends ping at regular intervals; if no pong received within timeout,
triggers reconnection or alert.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional


class HeartbeatManager:
    """Manages heartbeat ping/pong to detect connection loss.

    Sends heartbeat messages at configurable intervals.
    If no response arrives within the timeout period, triggers
    the on_timeout callback (typically reconnect or alert).

    The heartbeat message format:
      {"type": "heartbeat", "seq": <int>, "timestamp": <float>}

    The robot should echo back the same seq number.
    """

    DEFAULT_INTERVAL = 1.0     # seconds between heartbeats
    DEFAULT_TIMEOUT = 3.0      # seconds without response → timeout
    MAX_MISSED_BEATS = 3       # consecutive missed → connection lost

    def __init__(self):
        self._interval = self.DEFAULT_INTERVAL
        self._timeout = self.DEFAULT_TIMEOUT
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._seq = 0
        self._last_ack_seq = 0
        self._last_response_time: float = 0
        self._missed_count = 0
        self._lock = threading.Lock()
        self._send_func: Optional[Callable[[dict], bool]] = None

        self._on_timeout: Optional[Callable[[], None]] = None
        self._on_beat: Optional[Callable[[float], None]] = None  # latency callback

    def set_send_func(self, func: Callable[[dict], bool]) -> None:
        """Set the function used to send heartbeat messages."""
        self._send_func = func

    def on_timeout(self, callback: Callable[[], None]) -> None:
        """Called when heartbeat times out (connection likely lost)."""
        self._on_timeout = callback

    def on_beat(self, callback: Callable[[float], None]) -> None:
        """Called with latency (seconds) on each successful round-trip."""
        self._on_beat = callback

    def start(self, interval_s: float = DEFAULT_INTERVAL) -> None:
        self._interval = interval_s
        self._running = True
        self._last_response_time = time.time()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    def on_pong(self, seq: int) -> None:
        """Call this when a heartbeat response is received."""
        with self._lock:
            if seq >= self._last_ack_seq:
                self._last_ack_seq = seq
                now = time.time()
                latency = now - self._last_response_time
                self._last_response_time = now
                self._missed_count = 0
                if self._on_beat:
                    self._on_beat(latency)

    def is_alive(self) -> bool:
        with self._lock:
            return self._missed_count < self.MAX_MISSED_BEATS

    # ── Internal ────────────────────────────────────────────────

    def _loop(self) -> None:
        while self._running:
            time.sleep(self._interval)

            # Check timeout
            with self._lock:
                elapsed = time.time() - self._last_response_time
                if elapsed > self._timeout:
                    self._missed_count += 1

            if self._missed_count >= self.MAX_MISSED_BEATS:
                if self._on_timeout:
                    self._on_timeout()
                break

            # Send heartbeat
            if self._send_func:
                self._seq += 1
                self._send_func({
                    "type": "heartbeat",
                    "seq": self._seq,
                    "timestamp": time.time(),
                })
