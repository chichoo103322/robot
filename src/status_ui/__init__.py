"""Member B — Status Management & Control UI.

Public API:
    StatusManager  — central state store (robot status, logs)
    LogSystem      — structured file+console logging
    ControlPanel   — control button logic and state bridge
    RobotDashboard — PyQt6 dashboard window
"""

from .status_manager import StatusManager
from .log_system import LogSystem
from .control_panel import ControlPanel
from .dashboard import RobotDashboard
