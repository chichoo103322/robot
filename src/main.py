"""Main entry point — wires all modules together.

Architecture:
  Member A (task_planner) → plans tasks & actions
  Member C (communication) → sends commands, receives status
  Member B (status_ui)    → displays status, logs, control UI

This file demonstrates the full integration. Replace simulation stubs
with actual robot connection for real hardware use.
"""

from __future__ import annotations

import time

from .common.enums import ActionType, TaskPriority
from .common.models import Action, RobotStatus
from .task_planner import (
    TaskManager,
    ActionScheduler,
    MotionPlanner,
    ObstacleDetector,
    AvoidancePlanner,
)
from .communication import APIService
from .status_ui import StatusManager, LogSystem, ControlPanel


def build_system():
    """Create and wire all system components.

    Returns a dict with all modules so the caller can start/stop.
    """
    # ── Create instances ────────────────────────────────────────
    log = LogSystem(log_dir="./logs")
    status_mgr = StatusManager()
    comm = APIService()
    motion_planner = MotionPlanner()
    obstacle_detector = ObstacleDetector()
    avoidance_planner = AvoidancePlanner(obstacle_detector, motion_planner)
    task_mgr = TaskManager()
    action_scheduler = ActionScheduler(comm)
    control_panel = ControlPanel(status_mgr)

    # ── Wire callbacks ──────────────────────────────────────────

    # C → B: incoming status → status manager → UI
    comm.on_status_received(status_mgr.update_robot_status)
    comm.on_sensor_data(lambda data: log.info("sensor", f"Received sensor data"))

    # C → A: incoming sensor data → obstacle detector
    comm.on_sensor_data(lambda data: obstacle_detector.detect(data))

    # A → C: action scheduler dispatches via comm
    action_scheduler.set_communication(comm)

    # A → B: log task events
    task_mgr.add_listener(
        lambda task, event: status_mgr.add_log("INFO", "task_planner",
                                                f"Task '{task.name}' {event}")
    )

    # C → A: action completion from robot
    comm.on_action_complete(action_scheduler.mark_action_complete)

    # Heartbeat timeout → reconnect
    comm.on_heartbeat_timeout(lambda: log.warning("comm", "Heartbeat timeout — connection lost"))

    # Control panel button callbacks
    control_panel.on_start_task = lambda tid: task_mgr.start_task(tid)
    control_panel.on_stop_task = lambda tid: task_mgr.stop_task(tid)
    control_panel.on_pause_task = lambda tid: task_mgr.pause_task(tid)
    control_panel.on_resume_task = lambda tid: task_mgr.resume_task(tid)
    control_panel.on_send_action = lambda at, params: action_scheduler.schedule_action(
        Action(action_type=at, params=params))

    return {
        "log": log,
        "status_mgr": status_mgr,
        "comm": comm,
        "motion_planner": motion_planner,
        "obstacle_detector": obstacle_detector,
        "avoidance_planner": avoidance_planner,
        "task_mgr": task_mgr,
        "action_scheduler": action_scheduler,
        "control_panel": control_panel,
    }


def demo_task_sequence(system: dict):
    """Create and run a demo task sequence with all 5+ action types."""
    mp = system["motion_planner"]
    task_mgr = system["task_mgr"]
    log = system["log"]

    # Build a task with 5+ action types
    actions = mp.build_action_sequence([
        {"type": "walk_straight", "distance": 2.0, "speed": 0.5},
        {"type": "turn_in_place", "angle": 90},
        {"type": "walk_straight", "distance": 1.0},
        {"type": "turn_walk", "distance": 1.5, "angle": 45},
        {"type": "stop"},
    ])

    task = task_mgr.create_task(
        name="Demo: 5-action sequence",
        actions=actions,
        priority=TaskPriority.NORMAL,
    )

    log.info("main", f"Created task '{task.name}' with {len(actions)} actions")
    log.info("main", "Actions: " + " → ".join(a.action_type.value for a in actions))

    task_mgr.start_task(task.task_id)
    return task


def run_headless(host: str = "127.0.0.1", port: int = 9090):
    """Run the system in headless mode (no GUI). Perfect for simulation."""
    system = build_system()
    log = system["log"]
    comm = system["comm"]

    log.info("main", "Starting robot control system (headless mode)")

    # Connect to robot/simulator
    if not comm.connect(host, port):
        log.warning("main", f"Could not connect to {host}:{port} — running offline")
    else:
        log.info("main", f"Connected to {host}:{port}")

    # Create demo task
    task = demo_task_sequence(system)

    try:
        while True:
            time.sleep(1)
            status = system["status_mgr"].get_robot_status()
            if status.error_code:
                log.error("main", f"Robot error: {status.error_code}")
    except KeyboardInterrupt:
        log.info("main", "Shutting down...")
        comm.disconnect()


def run_with_ui(host: str = "127.0.0.1", port: int = 9090):
    """Run the system with the PyQt6 dashboard UI."""
    system = build_system()
    log = system["log"]
    comm = system["comm"]
    dashboard = system.get("dashboard")

    log.info("main", "Starting robot control system (UI mode)")

    # Connect to robot/simulator
    comm.connect(host, port)

    # Import and launch dashboard
    from .status_ui import RobotDashboard
    dashboard = RobotDashboard(system["status_mgr"])
    dashboard.register_control_panel(system["control_panel"])

    # Create demo task
    demo_task_sequence(system)

    # Blocking — runs until window closes
    dashboard.run()
    comm.disconnect()


def run_with_web(web_host: str = "0.0.0.0", web_port: int = 8080,
                 robot_host: str = "127.0.0.1", robot_port: int = 9090):
    """Run the system with the web-based dashboard UI."""
    from .web_server import run_web
    run_web(web_host, web_port, robot_host, robot_port)


if __name__ == "__main__":
    import sys
    mode = sys.argv[1] if len(sys.argv) > 1 else "headless"
    host = sys.argv[2] if len(sys.argv) > 2 else "127.0.0.1"
    port = int(sys.argv[3]) if len(sys.argv) > 3 else 9090

    if mode == "ui":
        run_with_ui(host, port)
    elif mode == "web":
        web_port = int(sys.argv[2]) if len(sys.argv) > 2 else 8080
        robot_host = sys.argv[3] if len(sys.argv) > 3 else "127.0.0.1"
        robot_port = int(sys.argv[4]) if len(sys.argv) > 4 else 9090
        run_with_web("0.0.0.0", web_port, robot_host, robot_port)
    else:
        run_headless(host, port)
