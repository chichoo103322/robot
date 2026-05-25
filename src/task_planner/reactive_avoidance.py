"""ReactiveAvoidance — Member A

Core avoidance controller that runs during task execution. Monitors the
robot's forward path in real-time and automatically handles obstacles:

  1. Task running → robot walking forward
  2. Obstacle detected in path → emergency stop
  3. Pause current task → calculate avoidance route
  4. Execute avoidance (sidestep → go past → sidestep back)
  5. Resume original task → robot continues to target

This implements the "规划—执行—反馈" (plan-execute-feedback) closed loop
required by the PRD.
"""

from __future__ import annotations

import threading
import time
from enum import Enum
from typing import Callable, Optional

from ..common.enums import ActionType, ActionStatus
from ..common.models import Action, Task
from .motion_planner import MotionPlanner
from .vision_detector import VisionDetector


class AvoidanceState(Enum):
    IDLE = "idle"               # No task running, not monitoring
    MONITORING = "monitoring"   # Task running, watching for obstacles
    AVOIDING = "avoiding"       # Obstacle detected, executing avoidance
    RECOVERING = "recovering"   # Avoidance done, resuming original task


class ReactiveAvoidance:
    """Real-time reactive obstacle avoidance during task execution.

    Runs as a background thread. When an obstacle is detected in the
    robot's forward path, it:

    1. Immediately interrupts the current motion
    2. Plans a detour around the obstacle
    3. Executes the detour as injected actions
    4. Resumes the original task from where it left off
    5. Tracks remaining distance to ensure the robot reaches target

    Usage:
        ra = ReactiveAvoidance(vision_detector, motion_planner)
        ra.set_callbacks(
            on_interrupt=action_scheduler.interrupt_current_action,
            on_schedule=action_scheduler.schedule_action,
            on_task_pause=task_mgr.pause_task,
            on_task_resume=task_mgr.resume_task,
        )
        ra.start()
        # System runs... avoidance happens automatically
    """

    # Tuning parameters
    CHECK_INTERVAL = 0.1        # seconds — how often to scan for obstacles
    LOOKAHEAD_DISTANCE = 2.0    # meters — how far ahead to check
    AVOIDANCE_SIDESTEP = 0.6    # meters — lateral clearance for detour
    AVOIDANCE_FORWARD = 1.2     # meters — forward distance to clear obstacle
    MAX_RECOVERY_RETRIES = 3    # times to retry after avoidance

    def __init__(self, vision_detector: Optional[VisionDetector] = None,
                 motion_planner: Optional[MotionPlanner] = None):
        self.vision = vision_detector or VisionDetector(mode="simulated")
        self.motion = motion_planner or MotionPlanner()

        self._state = AvoidanceState.IDLE
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Original task tracking
        self._original_action: Optional[Action] = None
        self._remaining_distance: float = 0.0
        self._recovery_retries: int = 0

        # Callbacks — wired by the system integrator
        self._on_interrupt: Optional[Callable[[], bool]] = None
        self._on_schedule: Optional[Callable[[Action], str]] = None
        self._on_task_pause: Optional[Callable[[str], bool]] = None
        self._on_task_resume: Optional[Callable[[str], bool]] = None
        self._on_log: Optional[Callable[[str, str], None]] = None  # (level, msg)
        self._current_task_id: str = ""

        # Statistics
        self.avoidance_count: int = 0
        self.last_avoidance_time: float = 0

    # ── Callback setters ─────────────────────────────────────────

    def set_callbacks(self,
                      on_interrupt: Callable[[], bool],
                      on_schedule: Callable[[Action], str],
                      on_task_pause: Callable[[str], bool],
                      on_task_resume: Callable[[str], bool],
                      on_log: Optional[Callable[[str, str], None]] = None):
        """Wire the avoidance controller into the system."""
        self._on_interrupt = on_interrupt
        self._on_schedule = on_schedule
        self._on_task_pause = on_task_pause
        self._on_task_resume = on_task_resume
        self._on_log = on_log

    # ── Lifecycle ────────────────────────────────────────────────

    def start(self) -> None:
        """Start the background monitoring thread."""
        self._running = True
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._thread.start()
        self._log("INFO", "ReactiveAvoidance monitor started")

    def stop(self) -> None:
        self._running = False
        self._state = AvoidanceState.IDLE

    # ── Called by TaskManager when a task starts/stops ───────────

    def on_task_started(self, task_id: str, action: Optional[Action] = None) -> None:
        """Notify the monitor that a task has started executing."""
        with self._lock:
            self._current_task_id = task_id
            self._original_action = action
            self._recovery_retries = 0
            # Track remaining distance from action params
            if action and "distance_m" in action.params:
                self._remaining_distance = action.params["distance_m"]
            elif action and action.action_type in (
                ActionType.WALK_STRAIGHT, ActionType.WALK_BACKWARD,
                ActionType.TURN_WALK,
            ):
                self._remaining_distance = action.params.get("distance_m", 1.0)
            self._state = AvoidanceState.MONITORING
        self._log("INFO", f"Monitoring task {task_id} for obstacles")

    def on_task_stopped(self) -> None:
        with self._lock:
            self._state = AvoidanceState.IDLE
            self._original_action = None

    # ── Add simulated obstacles (dev/testing) ────────────────────

    def add_test_obstacle(self, x: float, y: float, radius: float = 0.3) -> None:
        """Add a test obstacle ahead of the robot for verification."""
        self.vision.add_simulated_obstacle(x, y, radius)

    def clear_test_obstacles(self) -> None:
        self.vision.clear_simulated()

    # ── Status ───────────────────────────────────────────────────

    @property
    def state(self) -> AvoidanceState:
        with self._lock:
            return self._state

    @property
    def is_avoiding(self) -> bool:
        return self._state == AvoidanceState.AVOIDING

    # ── Internal: monitoring loop ────────────────────────────────

    def _monitor_loop(self) -> None:
        """Background loop: check for obstacles, trigger avoidance if needed."""
        while self._running:
            time.sleep(self.CHECK_INTERVAL)

            with self._lock:
                if self._state != AvoidanceState.MONITORING:
                    continue

            # Run detection
            obstacles = self.vision.detect()

            # Check if path is blocked
            if not self.vision.is_path_clear(self.LOOKAHEAD_DISTANCE):
                nearest = self.vision.get_nearest_blocking_obstacle(
                    self.LOOKAHEAD_DISTANCE)
                if nearest:
                    self._handle_obstacle(nearest)

    def _handle_obstacle(self, obstacle: dict) -> None:
        """Obstacle detected! Execute avoidance sequence."""
        with self._lock:
            if self._state != AvoidanceState.MONITORING:
                return
            self._state = AvoidanceState.AVOIDING

        ox, oy = obstacle["center"]
        r = obstacle["radius"]
        self._log("WARNING",
                   f"Obstacle at ({ox:.2f}, {oy:.2f}) r={r:.2f}m — avoiding!")

        # Step 1: Emergency interrupt current action
        if self._on_interrupt:
            self._on_interrupt()
        time.sleep(0.05)

        # Step 2: Plan avoidance actions
        avoidance_actions = self._plan_avoidance(obstacle)

        # Step 3: Execute avoidance actions sequentially
        for action in avoidance_actions:
            if self._on_schedule:
                self._on_schedule(action)
            # Wait for action to complete (in real system, this would
            # be callback-driven; here we use a simple delay estimate)
            time.sleep(self._estimate_action_duration(action))

        # Step 4: Resume original task
        self.avoidance_count += 1
        self.last_avoidance_time = time.time()

        # Create a continuation action for the remaining distance
        with self._lock:
            remaining = max(0, self._remaining_distance -
                           (obstacle["center"][1] - obstacle["radius"]))
            if remaining > 0.1:
                resume_action = Action(
                    action_type=ActionType.WALK_STRAIGHT,
                    params={"distance_m": remaining, "speed": 0.5},
                )
                if self._on_schedule:
                    self._on_schedule(resume_action)
                self._log("INFO", f"Resuming: walk straight {remaining:.2f}m")
            self._state = AvoidanceState.RECOVERING

        # Transition back to monitoring
        time.sleep(0.2)
        with self._lock:
            if self._state == AvoidanceState.RECOVERING:
                self._state = AvoidanceState.MONITORING

    def _plan_avoidance(self, obstacle: dict) -> list[Action]:
        """Generate a 3-step avoidance maneuver around the obstacle.

        Route: sidestep → go past → sidestep back
        """
        ox = obstacle["center"][0]
        clearance = obstacle["radius"] + self.AVOIDANCE_SIDESTEP

        # Decide direction: sidestep to the side with more space
        direction = 1.0 if ox > 0 else -1.0

        actions = [
            # Step 1: Sidestep away from obstacle
            self.motion.plan_sidestep(clearance * direction),
            # Step 2: Walk forward past the obstacle
            self.motion.plan_straight_walk(self.AVOIDANCE_FORWARD),
            # Step 3: Sidestep back to original line
            self.motion.plan_sidestep(-clearance * direction),
        ]
        return actions

    @staticmethod
    def _estimate_action_duration(action: Action) -> float:
        """Rough duration estimate for waiting during avoidance."""
        if action.action_type == ActionType.SIDESTEP:
            dist = abs(action.params.get("distance_m", 0.5))
            return dist / 0.3 + 0.2  # sidestep is slower
        elif action.action_type == ActionType.WALK_STRAIGHT:
            dist = action.params.get("distance_m", 1.0)
            speed = action.params.get("speed", 0.5)
            return dist / speed + 0.3
        elif action.action_type == ActionType.STOP:
            return 0.1
        return 0.5

    def _log(self, level: str, message: str) -> None:
        if self._on_log:
            try:
                self._on_log(level, message)
            except Exception:
                pass
