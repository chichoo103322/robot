"""ActionScheduler — Member A

Orchestrates action execution: dispatches actions through the communication
layer, handles action switching, enforces priority preemption, and tracks
execution status for each action.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

from ..common.enums import ActionStatus, TaskPriority
from ..common.interfaces import IActionScheduler, ICommunication
from ..common.models import Action


class ActionScheduler(IActionScheduler):
    """Schedules and dispatches individual actions to the robot.

    Connects to Member C's ICommunication to send commands.
    Supports:
    - Priority-based preemption (EMERGENCY > HIGH > NORMAL > LOW)
    - Current action tracking
    - Completion callbacks
    """

    def __init__(self, comm: Optional[ICommunication] = None):
        self._comm = comm
        self._current_action: Optional[Action] = None
        self._action_map: dict[str, Action] = {}
        self._on_complete_callbacks: list[Callable[[Action], None]] = []
        self._lock = threading.Lock()

    def set_communication(self, comm: ICommunication) -> None:
        """Bind the communication layer (Member C)."""
        self._comm = comm

    # ── Scheduling ──────────────────────────────────────────────

    def schedule_action(self, action: Action) -> str:
        """Queue an action for execution. May preempt current action."""
        with self._lock:
            self._action_map[action.action_id] = action

            # Check if we should preempt
            if self._current_action and self._current_action.status == ActionStatus.RUNNING:
                if action.priority.value > self._current_action.priority.value:
                    self._preempt_current(action)

            # Execute now if nothing is running
            if self._current_action is None or self._current_action.status != ActionStatus.RUNNING:
                self._dispatch(action)

            return action.action_id

    def interrupt_current_action(self) -> bool:
        with self._lock:
            if self._current_action and self._current_action.status == ActionStatus.RUNNING:
                self._current_action.status = ActionStatus.INTERRUPTED
                # Send STOP command immediately
                if self._comm:
                    from ..common.models import Command
                    self._comm.send_command(Command(
                        action_type=self._current_action.action_type,
                        params={"emergency": True},
                    ))
                return True
            return False

    def get_current_action(self) -> Optional[Action]:
        with self._lock:
            return self._current_action

    def get_action_status(self, action_id: str) -> Optional[Action]:
        with self._lock:
            return self._action_map.get(action_id)

    # ── Completion handling ─────────────────────────────────────

    def on_action_complete(self, callback: Callable[[Action], None]) -> None:
        self._on_complete_callbacks.append(callback)

    def mark_action_complete(self, action_id: str, success: bool = True,
                              error_msg: str = "") -> None:
        """Called when communication layer reports action completion."""
        with self._lock:
            action = self._action_map.get(action_id)
            if action is None:
                return
            action.status = ActionStatus.COMPLETED if success else ActionStatus.FAILED
            action.completed_at = time.time()
            action.error_msg = error_msg

            if self._current_action and self._current_action.action_id == action_id:
                self._current_action = None

        for cb in self._on_complete_callbacks:
            try:
                cb(action)
            except Exception:
                pass

    # ── Internal ────────────────────────────────────────────────

    def _dispatch(self, action: Action) -> None:
        """Send action command to the robot via communication layer."""
        action.status = ActionStatus.RUNNING
        action.started_at = time.time()
        self._current_action = action

        if self._comm:
            from ..common.models import Command
            cmd = Command(
                action_type=action.action_type,
                params=action.params,
            )
            self._comm.send_command(cmd)

    def _preempt_current(self, new_action: Action) -> None:
        """Preempt the current action for a higher-priority one."""
        if self._current_action:
            self._current_action.status = ActionStatus.INTERRUPTED
        if self._comm:
            from ..common.models import Command
            from ..common.enums import ActionType
            self._comm.send_command(Command(action_type=ActionType.STOP, params={"emergency": True}))
