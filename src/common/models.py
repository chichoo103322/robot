"""Shared data models — the contract between all team members.

All models use Pydantic for serialization/validation so each module
can serialize to JSON for communication or UI display.
"""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field
from typing import Optional

from .enums import ActionType, ActionStatus, RobotState, TaskPriority


# ── Action ──────────────────────────────────────────────────────────

@dataclass
class Action:
    """A single robot action (e.g. walk straight for 2 meters)."""
    action_type: ActionType
    params: dict = field(default_factory=dict)
    priority: TaskPriority = TaskPriority.NORMAL

    # Auto-generated
    action_id: str = field(default_factory=lambda: uuid.uuid4().hex[:8])
    status: ActionStatus = ActionStatus.PENDING
    created_at: float = field(default_factory=time.time)

    # Execution feedback
    started_at: Optional[float] = None
    completed_at: Optional[float] = None
    error_msg: str = ""

    def to_dict(self) -> dict:
        return {
            "action_id": self.action_id,
            "action_type": self.action_type.value,
            "params": self.params,
            "priority": self.priority.value,
            "status": self.status.value,
            "created_at": self.created_at,
            "started_at": self.started_at,
            "completed_at": self.completed_at,
            "error_msg": self.error_msg,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Action":
        return cls(
            action_id=d.get("action_id", ""),
            action_type=ActionType(d["action_type"]),
            params=d.get("params", {}),
            priority=TaskPriority(d.get("priority", 1)),
            status=ActionStatus(d.get("status", "pending")),
            created_at=d.get("created_at", time.time()),
            started_at=d.get("started_at"),
            completed_at=d.get("completed_at"),
            error_msg=d.get("error_msg", ""),
        )


# ── Task ────────────────────────────────────────────────────────────

@dataclass
class Task:
    """A task composed of multiple actions executed in sequence."""
    name: str
    actions: list[Action] = field(default_factory=list)
    priority: TaskPriority = TaskPriority.NORMAL
    repeat: int = 1  # 0 = infinite loop

    task_id: str = field(default_factory=lambda: uuid.uuid4().hex[:8])
    current_action_index: int = 0
    is_running: bool = False
    created_at: float = field(default_factory=time.time)

    def to_dict(self) -> dict:
        return {
            "task_id": self.task_id,
            "name": self.name,
            "actions": [a.to_dict() for a in self.actions],
            "priority": self.priority.value,
            "repeat": self.repeat,
            "current_action_index": self.current_action_index,
            "is_running": self.is_running,
            "created_at": self.created_at,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Task":
        return cls(
            task_id=d.get("task_id", ""),
            name=d["name"],
            actions=[Action.from_dict(a) for a in d.get("actions", [])],
            priority=TaskPriority(d.get("priority", 1)),
            repeat=d.get("repeat", 1),
            current_action_index=d.get("current_action_index", 0),
            is_running=d.get("is_running", False),
            created_at=d.get("created_at", time.time()),
        )


# ── Command (to robot) ──────────────────────────────────────────────

@dataclass
class Command:
    """Command sent from upper system to robot controller via Member C."""
    command_id: str = field(default_factory=lambda: uuid.uuid4().hex[:8])
    action_type: ActionType = ActionType.STOP
    params: dict = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> dict:
        return {
            "command_id": self.command_id,
            "action_type": self.action_type.value,
            "params": self.params,
            "timestamp": self.timestamp,
        }

    def to_json(self) -> str:
        import json
        return json.dumps(self.to_dict())

    @classmethod
    def from_dict(cls, d: dict) -> "Command":
        return cls(
            command_id=d.get("command_id", ""),
            action_type=ActionType(d["action_type"]),
            params=d.get("params", {}),
            timestamp=d.get("timestamp", time.time()),
        )

    @classmethod
    def from_json(cls, s: str) -> "Command":
        import json
        return cls.from_dict(json.loads(s))


# ── RobotStatus (from robot) ────────────────────────────────────────

@dataclass
class RobotStatus:
    """Status data received from the robot."""
    state: RobotState = RobotState.IDLE
    battery: float = 100.0
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: float = 0.0
    current_action_id: str = ""
    error_code: int = 0
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> dict:
        return {
            "state": self.state.value,
            "battery": self.battery,
            "position": list(self.position),
            "orientation": list(self.orientation),
            "velocity": self.velocity,
            "current_action_id": self.current_action_id,
            "error_code": self.error_code,
            "timestamp": self.timestamp,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "RobotStatus":
        return cls(
            state=RobotState(d.get("state", "idle")),
            battery=d.get("battery", 100.0),
            position=tuple(d.get("position", [0, 0, 0])),
            orientation=tuple(d.get("orientation", [0, 0, 0])),
            velocity=d.get("velocity", 0.0),
            current_action_id=d.get("current_action_id", ""),
            error_code=d.get("error_code", 0),
            timestamp=d.get("timestamp", time.time()),
        )


# ── SensorData (vision / obstacle) ──────────────────────────────────

@dataclass
class SensorData:
    """Raw sensor/vision data for obstacle detection."""
    depth_map: Optional[list] = None       # 2D depth array
    rgb_frame: Optional[bytes] = None      # Camera frame bytes
    lidar_points: list[tuple[float, float, float]] = field(default_factory=list)
    imu: dict = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> dict:
        return {
            "lidar_points": [list(p) for p in self.lidar_points],
            "imu": self.imu,
            "timestamp": self.timestamp,
            # depth_map and rgb_frame are too large — send via separate channel
        }

    @classmethod
    def from_dict(cls, d: dict) -> "SensorData":
        return cls(
            lidar_points=[tuple(p) for p in d.get("lidar_points", [])],
            imu=d.get("imu", {}),
            timestamp=d.get("timestamp", time.time()),
        )
