"""Abstract interfaces — the contract each team member implements.

Member A → ITaskManager, IActionScheduler, IMotionPlanner
Member B → IStatusManager (and UI callbacks)
Member C → ICommunication
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Callable, Optional

from .models import Action, Command, RobotStatus, SensorData, Task


# ── Member A interfaces ─────────────────────────────────────────────

class ITaskManager(ABC):
    """Task lifecycle: create, queue, start, stop, get status."""

    @abstractmethod
    def create_task(self, name: str, actions: list[Action], repeat: int = 1) -> Task:
        ...

    @abstractmethod
    def start_task(self, task_id: str) -> bool:
        ...

    @abstractmethod
    def stop_task(self, task_id: str) -> bool:
        ...

    @abstractmethod
    def pause_task(self, task_id: str) -> bool:
        ...

    @abstractmethod
    def resume_task(self, task_id: str) -> bool:
        ...

    @abstractmethod
    def get_task(self, task_id: str) -> Optional[Task]:
        ...

    @abstractmethod
    def get_all_tasks(self) -> list[Task]:
        ...

    @abstractmethod
    def get_current_task(self) -> Optional[Task]:
        ...


class IActionScheduler(ABC):
    """Action execution orchestration: dispatch, switch, preempt."""

    @abstractmethod
    def schedule_action(self, action: Action) -> str:  # returns action_id
        ...

    @abstractmethod
    def interrupt_current_action(self) -> bool:
        ...

    @abstractmethod
    def get_current_action(self) -> Optional[Action]:
        ...

    @abstractmethod
    def on_action_complete(self, callback: Callable[[Action], None]) -> None:
        ...

    @abstractmethod
    def get_action_status(self, action_id: str) -> Optional[Action]:
        ...


class IMotionPlanner(ABC):
    """Motion sequence planning and trajectory generation."""

    @abstractmethod
    def plan_straight_walk(self, distance_m: float, speed: float = 0.5) -> Action:
        ...

    @abstractmethod
    def plan_turn_in_place(self, angle_deg: float, angular_speed: float = 30.0) -> Action:
        ...

    @abstractmethod
    def plan_turn_walk(self, distance_m: float, angle_deg: float, speed: float = 0.5) -> Action:
        ...

    @abstractmethod
    def plan_stop(self) -> Action:
        ...

    @abstractmethod
    def plan_avoidance_path(self, obstacles: list[dict], current_pos: tuple) -> list[Action]:
        ...

    @abstractmethod
    def build_action_sequence(self, actions: list[dict]) -> list[Action]:
        """Build a sequence of actions from high-level descriptions.

        Each dict: {"type": "walk_straight", "distance": 2.0, "speed": 0.5}
        """
        ...


# ── Member B interface ──────────────────────────────────────────────

class IStatusManager(ABC):
    """Central state store — written by Member C on data receive,
    read by Member B's UI for display."""

    @abstractmethod
    def update_robot_status(self, status: RobotStatus) -> None:
        ...

    @abstractmethod
    def get_robot_status(self) -> RobotStatus:
        ...

    @abstractmethod
    def add_log(self, level: str, source: str, message: str) -> None:
        ...

    @abstractmethod
    def get_logs(self, count: int = 100) -> list[dict]:
        ...

    @abstractmethod
    def subscribe_status(self, callback: Callable[[RobotStatus], None]) -> None:
        ...


# ── Member C interface ──────────────────────────────────────────────

class ICommunication(ABC):
    """Communication layer between upper system and robot controller."""

    @abstractmethod
    def connect(self, host: str, port: int) -> bool:
        ...

    @abstractmethod
    def disconnect(self) -> None:
        ...

    @abstractmethod
    def send_command(self, command: Command) -> bool:
        ...

    @abstractmethod
    def start_receiving(self) -> None:
        """Start background thread/async loop for receiving status data."""

    @abstractmethod
    def on_status_received(self, callback: Callable[[RobotStatus], None]) -> None:
        ...

    @abstractmethod
    def on_sensor_data(self, callback: Callable[[SensorData], None]) -> None:
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        ...

    @abstractmethod
    def start_heartbeat(self, interval_s: float = 1.0) -> None:
        ...
