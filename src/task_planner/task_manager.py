"""TaskManager — Member A

Creates and manages robot action tasks, maintains task queue,
handles task lifecycle (create → start → pause → resume → stop).
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Callable, Optional

from ..common.enums import ActionStatus, TaskPriority
from ..common.interfaces import ITaskManager
from ..common.models import Action, Task


class TaskManager(ITaskManager):
    """Manages the full lifecycle of robot tasks.

    Tasks are queued by priority and executed sequentially.
    Supports pause/resume/stop and notifies listeners on state changes.
    """

    def __init__(self):
        self._tasks: dict[str, Task] = {}
        self._queue: deque[str] = deque()  # task_ids in execution order
        self._current_task: Optional[Task] = None
        self._lock = threading.Lock()
        self._listeners: list[Callable[[Task, str], None]] = []  # (task, event)

    # ── Task CRUD ───────────────────────────────────────────────

    def create_task(self, name: str, actions: list[Action],
                    priority: TaskPriority = TaskPriority.NORMAL,
                    repeat: int = 1) -> Task:
        task = Task(
            name=name,
            actions=actions,
            priority=priority,
            repeat=repeat,
        )
        with self._lock:
            self._tasks[task.task_id] = task
            self._enqueue_by_priority(task)
        self._notify(task, "created")
        return task

    def get_task(self, task_id: str) -> Optional[Task]:
        with self._lock:
            return self._tasks.get(task_id)

    def get_all_tasks(self) -> list[Task]:
        with self._lock:
            return list(self._tasks.values())

    def get_current_task(self) -> Optional[Task]:
        with self._lock:
            return self._current_task

    # ── Task control ────────────────────────────────────────────

    def start_task(self, task_id: str) -> bool:
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None:
                return False
            task.is_running = True
            task.current_action_index = 0
            self._current_task = task
        self._notify(task, "started")
        return True

    def stop_task(self, task_id: str) -> bool:
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None:
                return False
            task.is_running = False
            for action in task.actions:
                if action.status == ActionStatus.RUNNING:
                    action.status = ActionStatus.INTERRUPTED
            if self._current_task and self._current_task.task_id == task_id:
                self._current_task = None
        self._notify(task, "stopped")
        return True

    def pause_task(self, task_id: str) -> bool:
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None or not task.is_running:
                return False
            task.is_running = False
        self._notify(task, "paused")
        return True

    def resume_task(self, task_id: str) -> bool:
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None or task.is_running:
                return False
            task.is_running = True
        self._notify(task, "resumed")
        return True

    # ── Action progression ──────────────────────────────────────

    def advance_action(self, task_id: str) -> Optional[Action]:
        """Move to the next action in a task. Returns the next action or None if done."""
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None:
                return None
            idx = task.current_action_index
            if idx < len(task.actions):
                task.actions[idx].status = ActionStatus.COMPLETED
                task.actions[idx].completed_at = time.time()
            task.current_action_index += 1
            if task.current_action_index >= len(task.actions):
                if task.repeat == 0 or task.repeat > 1:
                    task.current_action_index = 0
                    if task.repeat > 1:
                        task.repeat -= 1
                else:
                    task.is_running = False
                    self._current_task = None
                    self._notify(task, "completed")
                    return None
            next_action = task.actions[task.current_action_index]
            return next_action

    def get_current_action(self, task_id: str) -> Optional[Action]:
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None:
                return None
            idx = task.current_action_index
            if 0 <= idx < len(task.actions):
                return task.actions[idx]
            return None

    # ── Internal ────────────────────────────────────────────────

    def _enqueue_by_priority(self, task: Task) -> None:
        """Insert task into queue sorted by priority (highest first)."""
        self._queue.append(task.task_id)
        # Re-sort by priority descending
        items = sorted(
            [(tid, self._tasks[tid].priority.value) for tid in self._queue
             if tid in self._tasks],
            key=lambda x: -x[1],
        )
        self._queue = deque(tid for tid, _ in items)

    def add_listener(self, callback: Callable[[Task, str], None]) -> None:
        """Register a callback for task lifecycle events (created, started, stopped, etc.)."""
        self._listeners.append(callback)

    def _notify(self, task: Task, event: str) -> None:
        for cb in self._listeners:
            try:
                cb(task, event)
            except Exception:
                pass
