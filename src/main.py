"""Main entry point -- wires all modules together.

Architecture (per PRD):
  Member A (task_planner) -> plans tasks, schedules actions, obstacle avoidance
  Member C (communication) -> sends commands, receives status
  Member B (status_ui)    -> status management, logging

This file demonstrates the full integration. Replace simulation stubs
with actual robot connection for real hardware use.
"""

from __future__ import annotations

import time

from .common.enums import TaskPriority
from .common.models import Action, RobotStatus
from .task_planner import (
    TaskManager,
    ActionScheduler,
    MotionPlanner,
    ObstacleDetector,
    AvoidancePlanner,
    VisionDetector,
    ReactiveAvoidance,
)
from .communication import APIService
from .status_ui import StatusManager, LogSystem


def build_system():
    """Create and wire all system components.

    Returns a dict with all modules so the caller can start/stop.
    """
    # -- Create instances --
    log = LogSystem(log_dir="./logs")
    status_mgr = StatusManager()
    comm = APIService()
    motion_planner = MotionPlanner()
    obstacle_detector = ObstacleDetector()
    avoidance_planner = AvoidancePlanner(obstacle_detector, motion_planner)
    vision_detector = VisionDetector(mode="simulated")
    task_mgr = TaskManager()
    action_scheduler = ActionScheduler(comm)

    # -- Reactive avoidance controller (Member A) --
    reactive_avoidance = ReactiveAvoidance(vision_detector, motion_planner)
    reactive_avoidance.set_callbacks(
        on_interrupt=lambda: action_scheduler.interrupt_current_action(),
        on_schedule=lambda a: action_scheduler.schedule_action(a),
        on_task_pause=lambda tid: task_mgr.pause_task(tid),
        on_task_resume=lambda tid: task_mgr.resume_task(tid),
        on_log=lambda lvl, msg: status_mgr.add_log(lvl, "avoidance", msg),
    )
    reactive_avoidance.start()

    # -- Wire callbacks --

    # C -> B: incoming status -> status manager
    comm.on_status_received(status_mgr.update_robot_status)
    comm.on_sensor_data(lambda data: log.info("sensor", f"Received sensor data"))

    # C -> A: incoming sensor data -> obstacle detector
    comm.on_sensor_data(lambda data: obstacle_detector.detect(data))

    # A -> C: action scheduler dispatches via comm
    action_scheduler.set_communication(comm)

    # A -> B: log task events + notify reactive avoidance
    def _on_task_event(task, event):
        status_mgr.add_log("INFO", "task_planner",
                           f"Task '{task.name}' {event}")
        if event == "started":
            action = task.actions[0] if task.actions else None
            reactive_avoidance.on_task_started(task.task_id, action)
        elif event in ("stopped", "completed"):
            reactive_avoidance.on_task_stopped()

    task_mgr.add_listener(_on_task_event)

    # C -> A: action completion from robot
    comm.on_action_complete(action_scheduler.mark_action_complete)

    # Heartbeat timeout -> reconnect handling
    comm.on_heartbeat_timeout(lambda: log.warning("comm", "Heartbeat timeout -- connection lost"))

    return {
        "log": log,
        "status_mgr": status_mgr,
        "comm": comm,
        "motion_planner": motion_planner,
        "obstacle_detector": obstacle_detector,
        "avoidance_planner": avoidance_planner,
        "vision_detector": vision_detector,
        "reactive_avoidance": reactive_avoidance,
        "task_mgr": task_mgr,
        "action_scheduler": action_scheduler,
    }


def demo_task_sequence(system: dict):
    """Create and run a demo task with obstacle avoidance test.

    Places a simulated obstacle 1.2m ahead. The robot walks 3m forward.
    At ~1.2m it encounters the obstacle -> auto-avoids -> continues to 3m.
    """
    mp = system["motion_planner"]
    task_mgr = system["task_mgr"]
    log = system["log"]

    # Add a test obstacle 1.2m ahead, slightly to the right
    if "vision_detector" in system:
        system["vision_detector"].add_simulated_obstacle(0.15, 1.2, 0.35)
        log.info("main", "Placed test obstacle at (0.15, 1.2m) r=0.35m")

    # Build a task: walk 3m forward (will trigger avoidance at ~1.2m)
    actions = mp.build_action_sequence([
        {"type": "walk_straight", "distance": 3.0, "speed": 0.5},
        {"type": "turn_in_place", "angle": 90},
        {"type": "walk_straight", "distance": 1.5},
        {"type": "turn_walk", "distance": 1.5, "angle": 45},
        {"type": "stop"},
    ])

    task = task_mgr.create_task(
        name="Demo: Obstacle Avoidance Test",
        actions=actions,
        priority=TaskPriority.NORMAL,
    )

    log.info("main", f"Created task '{task.name}' with {len(actions)} actions")
    log.info("main", "Actions: " + " -> ".join(a.action_type.value for a in actions))

    task_mgr.start_task(task.task_id)
    return task


def run_headless(host: str = "127.0.0.1", port: int = 9090):
    """Run the system in headless mode -- core task planning + communication.

    This is the primary execution mode per PRD. Connects to a robot controller
    (or mock server) via TCP and executes the demo task sequence with
    real-time obstacle avoidance.
    """
    system = build_system()
    log = system["log"]
    comm = system["comm"]

    log.info("main", "Starting robot control system (headless mode)")

    # Connect to robot/simulator
    if not comm.connect(host, port):
        log.warning("main", f"Could not connect to {host}:{port} -- running offline")
    else:
        log.info("main", f"Connected to {host}:{port}")

    # Create demo task
    demo_task_sequence(system)

    try:
        while True:
            time.sleep(1)
            status = system["status_mgr"].get_robot_status()
            if status.error_code:
                log.error("main", f"Robot error: {status.error_code}")
    except KeyboardInterrupt:
        log.info("main", "Shutting down...")
        comm.disconnect()


if __name__ == "__main__":
    import sys
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 9090
    run_headless(host, port)
