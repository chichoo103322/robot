"""Shared enums for the robot control system."""

from enum import Enum, auto


class ActionType(Enum):
    """Robot action types — minimum 5 required for acceptance."""
    WALK_STRAIGHT = "walk_straight"        # 直线行走
    TURN_IN_PLACE = "turn_in_place"        # 原地掉头
    TURN_WALK = "turn_walk"                # 转弯行走
    STOP = "stop"                          # 停止动作
    WALK_BACKWARD = "walk_backward"        # 后退行走
    SIDESTEP = "sidestep"                  # 侧向移动
    AVOID_OBSTACLE = "avoid_obstacle"      # 视觉避障自主绕行


class ActionStatus(Enum):
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    INTERRUPTED = "interrupted"


class RobotState(Enum):
    IDLE = "idle"
    MOVING = "moving"
    AVOIDING = "avoiding"
    STOPPED = "stopped"
    ERROR = "error"


class TaskPriority(Enum):
    LOW = 0
    NORMAL = 1
    HIGH = 2
    EMERGENCY = 3
