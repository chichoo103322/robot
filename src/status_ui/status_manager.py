"""StatusManager — Member B

Central state store for robot status, written by Member C on incoming data,
read by Member B's UI for display. Thread-safe singleton-like store.

Integrates with:
  - Member C: calls update_robot_status() when data arrives
  - Member B UI: reads status, subscribes to updates
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Callable

from ..common.enums import RobotState
from ..common.interfaces import IStatusManager
from ..common.models import RobotStatus


class StatusManager(IStatusManager):
    """Thread-safe central state store.

    Maintains:
      - Current robot status (position, battery, state, etc.)
      - Log buffer (last N entries)
      - Subscriber list for push updates to UI
    """

    MAX_LOGS = 500

    def __init__(self):
        self._status = RobotStatus()
        self._logs: deque[dict] = deque(maxlen=self.MAX_LOGS)
        self._subscribers: list[Callable[[RobotStatus], None]] = []
        self._lock = threading.Lock()

    # ── Status ──────────────────────────────────────────────────

    def update_robot_status(self, status: RobotStatus) -> None:
        with self._lock:
            self._status = status
        for cb in self._subscribers:
            try:
                cb(status)
            except Exception:
                pass

    def get_robot_status(self) -> RobotStatus:
        with self._lock:
            return self._status

    def subscribe_status(self, callback: Callable[[RobotStatus], None]) -> None:
        self._subscribers.append(callback)

    # ── Logs ────────────────────────────────────────────────────

    def add_log(self, level: str, source: str, message: str) -> None:
        entry = {
            "timestamp": time.time(),
            "level": level,
            "source": source,
            "message": message,
        }
        with self._lock:
            self._logs.append(entry)

    def get_logs(self, count: int = 100, level: str = "") -> list[dict]:
        with self._lock:
            logs = list(self._logs)[-count:]
        if level:
            logs = [l for l in logs if l["level"].upper() == level.upper()]
        return logs

    # ── Convenience ─────────────────────────────────────────────

    @property
    def is_robot_moving(self) -> bool:
        return self._status.state == RobotState.MOVING

    @property
    def is_robot_error(self) -> bool:
        return self._status.state == RobotState.ERROR

    @property
    def battery_level(self) -> float:
        return self._status.battery

    @property
    def position(self) -> tuple[float, float, float]:
        return self._status.position
