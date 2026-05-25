"""Member A — Task Planning, Action Scheduling & Obstacle Avoidance.

Public API:
    TaskManager      — create and manage robot tasks
    ActionScheduler  — dispatch and switch actions
    MotionPlanner    — generate action parameters
    ObstacleDetector — detect obstacles from sensor data
    AvoidancePlanner — plan avoidance paths
    VisionDetector   — camera-based obstacle detection (OpenCV/YOLO)
    ReactiveAvoidance— real-time monitoring + auto-avoidance during tasks
"""

from .task_manager import TaskManager
from .action_scheduler import ActionScheduler
from .motion_planner import MotionPlanner
from .obstacle_detector import ObstacleDetector
from .avoidance_planner import AvoidancePlanner
from .vision_detector import VisionDetector
from .reactive_avoidance import ReactiveAvoidance
