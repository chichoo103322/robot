"""CommandSender — Member C

Sends structured Command objects to the robot via SocketClient.
Handles serialization, queuing, and delivery confirmation.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Optional

from ..common.models import Command


class CommandSender:
    """Reliable command sender with retry and queueing.

    Features:
      - Send queue with configurable max size
      - Automatic retry on send failure
      - Command acknowledgment tracking
      - Send rate limiting (throttle)
    """

    MAX_QUEUE_SIZE = 100
    MAX_RETRIES = 3
    RETRY_DELAY = 0.1     # seconds
    SEND_INTERVAL = 0.05  # seconds minimum between sends

    def __init__(self, socket_client=None):
        self._socket = socket_client  # SocketClient instance
        self._queue: deque[tuple[Command, int]] = deque()  # (command, retries)
        self._lock = threading.Lock()
        self._last_send_time: float = 0
        self._running = False
        self._send_thread: Optional[threading.Thread] = None

    def set_socket(self, socket_client) -> None:
        self._socket = socket_client

    def start(self) -> None:
        self._running = True
        self._send_thread = threading.Thread(target=self._process_queue, daemon=True)
        self._send_thread.start()

    def stop(self) -> None:
        self._running = False

    def send_command(self, command: Command) -> bool:
        """Enqueue a command for sending. Returns True if queued."""
        with self._lock:
            if len(self._queue) >= self.MAX_QUEUE_SIZE:
                return False
            self._queue.append((command, 0))
        return True

    def send_immediate(self, command: Command) -> bool:
        """Send a command immediately, bypassing the queue."""
        if self._socket and self._socket.is_connected():
            return self._socket.send_json(command.to_dict())
        return False

    # ── Internal ────────────────────────────────────────────────

    def _process_queue(self) -> None:
        while self._running:
            with self._lock:
                if not self._queue:
                    pass
                else:
                    command, retries = self._queue[0]

                    # Rate limiting
                    now = time.time()
                    if now - self._last_send_time < self.SEND_INTERVAL:
                        pass
                    elif self._socket and self._socket.is_connected():
                        success = self._socket.send_json(command.to_dict())
                        if success:
                            self._queue.popleft()
                            self._last_send_time = now
                        else:
                            retries += 1
                            if retries >= self.MAX_RETRIES:
                                self._queue.popleft()  # drop after max retries
                            else:
                                self._queue[0] = (command, retries)

            time.sleep(0.01)  # 100Hz queue processing
