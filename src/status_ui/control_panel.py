"""ControlPanel — Member B

Qt-based control panel for the robot system.
Provides buttons for task control and displays robot status in real-time.

Key features:
  - Start/Stop/Pause/Resume task buttons
  - Real-time status display (position, battery, state)
  - Action progress indicator
  - Log viewer with filtering
  - Vision display area (for camera feed)
"""

from __future__ import annotations

import threading
import time
from typing import Optional

from ..common.enums import ActionType, RobotState
from ..common.interfaces import IStatusManager
from ..common.models import RobotStatus


class ControlPanel:
    """Backend logic for the control panel UI.

    The actual Qt widgets are created in RobotDashboard.
    This class provides the state and callbacks that the UI binds to.

    If PyQt6 is not available, this falls back to a headless mode
    (useful for testing or running on a headless robot computer).
    """

    def __init__(self, status_manager: Optional[IStatusManager] = None):
        self._status_manager = status_manager
        self._running = False
        self._lock = threading.Lock()

        # Task control callbacks — set by main app after wiring
        self.on_start_task: Optional[callable] = None
        self.on_stop_task: Optional[callable] = None
        self.on_pause_task: Optional[callable] = None
        self.on_resume_task: Optional[callable] = None
        self.on_send_action: Optional[callable] = None

    # ── Button actions ──────────────────────────────────────────

    def start_task(self, task_id: str) -> None:
        if self.on_start_task:
            self.on_start_task(task_id)

    def stop_task(self, task_id: str) -> None:
        if self.on_stop_task:
            self.on_stop_task(task_id)

    def pause_task(self, task_id: str) -> None:
        if self.on_pause_task:
            self.on_pause_task(task_id)

    def resume_task(self, task_id: str) -> None:
        if self.on_resume_task:
            self.on_resume_task(task_id)

    def send_manual_action(self, action_type: ActionType, params: dict = None) -> None:
        if self.on_send_action:
            self.on_send_action(action_type, params or {})

    # ── Status queries ──────────────────────────────────────────

    def get_status_text(self) -> dict:
        """Return key status fields for UI display."""
        if self._status_manager:
            s = self._status_manager.get_robot_status()
            return {
                "state": s.state.value,
                "battery": f"{s.battery:.0f}%",
                "position": f"({s.position[0]:.2f}, {s.position[1]:.2f}, {s.position[2]:.2f})",
                "velocity": f"{s.velocity:.2f} m/s",
                "error": s.error_code,
            }
        return {}

    def get_logs_for_display(self, count: int = 50) -> list[dict]:
        if self._status_manager:
            return self._status_manager.get_logs(count)
        return []
