"""AvoidancePlanner — Member A

Generates collision-free paths when obstacles are detected.
Integrates with ObstacleDetector and MotionPlanner to produce
avoidance action sequences that the ActionScheduler can execute.

Strategy:
  1. Receive obstacle list from ObstacleDetector
  2. Determine best avoidance direction (left/right)
  3. Generate sidestep + forward + return sidestep actions via MotionPlanner
  4. Return action sequence for scheduling
"""

from __future__ import annotations

from typing import Optional

from ..common.models import Action
from .motion_planner import MotionPlanner
from .obstacle_detector import ObstacleDetector


class AvoidancePlanner:
    """Plans obstacle avoidance paths for the robot.

    Uses a reactive approach:
    - If path is clear → proceed straight
    - If blocked → compute best avoidance direction → generate detour actions
    - After detour → resume original path

    Attributes:
        obstacle_detector: Reference to the ObstacleDetector instance
        motion_planner: Reference to the MotionPlanner instance
        avoidance_margin: Extra clearance beyond detected obstacle radius (m)
        forward_lookahead: How far ahead to check for obstacles (m)
    """

    AVOIDANCE_MARGIN = 0.2      # meters extra clearance
    FORWARD_LOOKAHEAD = 2.0     # meters — scan distance ahead

    def __init__(self, obstacle_detector: Optional[ObstacleDetector] = None,
                 motion_planner: Optional[MotionPlanner] = None):
        self.obstacle_detector = obstacle_detector or ObstacleDetector()
        self.motion_planner = motion_planner or MotionPlanner()

    # ── Main planning entry ─────────────────────────────────────

    def plan_avoidance(self, current_pos: tuple[float, float],
                       target_direction: tuple[float, float],
                       target_distance: float) -> list[Action]:
        """Plan an avoidance maneuver.

        Args:
            current_pos: (x, y) current position
            target_direction: (dx, dy) desired movement direction
            target_distance: how far we want to go

        Returns:
            List of Action objects for execution.
            If no obstacle, returns a single straight-walk action.
            If blocked, returns a multi-step avoidance sequence.
        """
        obstacles = self.obstacle_detector.get_obstacles()

        if not obstacles:
            # Path is clear — proceed normally
            return [self.motion_planner.plan_straight_walk(target_distance)]

        # Check if the forward path is blocked
        blocking = self._find_blocking_obstacles(obstacles, current_pos, target_direction)

        if not blocking:
            return [self.motion_planner.plan_straight_walk(target_distance)]

        # Plan avoidance around the nearest blocking obstacle
        nearest = blocking[0]
        avoidance_actions = self._generate_avoidance_actions(
            current_pos, nearest, target_direction, target_distance)

        return avoidance_actions

    # ── Internal planning logic ─────────────────────────────────

    def _find_blocking_obstacles(self, obstacles: list[dict],
                                  pos: tuple, direction: tuple) -> list[dict]:
        """Find obstacles that block movement in the given direction."""
        dx, dy = direction
        blocking: list[dict] = []

        for obs in obstacles:
            ox, oy = obs["center"]
            radius = obs["radius"] + self.AVOIDANCE_MARGIN

            # Vector from robot to obstacle
            rx = ox - pos[0]
            ry = oy - pos[1]

            # Distance to obstacle
            dist = (rx**2 + ry**2) ** 0.5

            if dist > self.FORWARD_LOOKAHEAD:
                continue

            # Project obstacle onto movement direction
            dir_mag = (dx**2 + dy**2) ** 0.5
            if dir_mag < 1e-6:
                continue
            proj = (rx * dx + ry * dy) / dir_mag

            # Obstacle is ahead (positive projection) and within the path width
            if proj > 0 and proj - radius < self.FORWARD_LOOKAHEAD:
                # Lateral distance (perpendicular to direction)
                lateral = abs(rx * (-dy) - ry * dx) / dir_mag
                if lateral < radius + 0.2:  # robot half-width approx
                    blocking.append({
                        **obs,
                        "distance_ahead": proj,
                        "lateral_offset": lateral,
                    })

        return sorted(blocking, key=lambda o: o["distance_ahead"])

    def _generate_avoidance_actions(self, pos: tuple,
                                     obstacle: dict,
                                     direction: tuple,
                                     target_distance: float) -> list[Action]:
        """Generate sidestep-based avoidance actions."""
        lateral_offset = obstacle.get("lateral_offset", 0)
        obs_radius = obstacle["radius"] + self.AVOIDANCE_MARGIN

        # Decide avoidance side: go to the side with more clearance
        clearance_needed = obs_radius * 2.0

        # Positive sidestep = right, negative = left
        # Default to the side with more lateral clearance
        sidestep_dir = 1.0 if lateral_offset > 0 else -1.0

        actions: list[Action] = []

        # Step 1: Sidestep to clear the obstacle
        actions.append(self.motion_planner.plan_sidestep(
            clearance_needed * sidestep_dir))

        # Step 2: Walk forward past the obstacle
        forward_distance = target_distance * 0.6
        actions.append(self.motion_planner.plan_straight_walk(forward_distance))

        # Step 3: Sidestep back to original line
        actions.append(self.motion_planner.plan_sidestep(
            -clearance_needed * sidestep_dir))

        # Step 4: Continue remaining distance
        remaining = target_distance - forward_distance
        if remaining > 0:
            actions.append(self.motion_planner.plan_straight_walk(remaining))

        return actions

    # ── Reactive avoidance (for continuous use) ──────────────────

    def should_avoid(self) -> bool:
        """Check if any obstacle is within the danger zone."""
        obstacles = self.obstacle_detector.get_obstacles()
        for obs in obstacles:
            dist = (obs["center"][0]**2 + obs["center"][1]**2) ** 0.5
            if dist < (obs["radius"] + self.AVOIDANCE_MARGIN + 0.5):
                return True
        return False

    def get_emergency_stop_if_needed(self) -> Optional[Action]:
        """Return an emergency STOP action if an obstacle is dangerously close."""
        obstacles = self.obstacle_detector.get_obstacles()
        for obs in obstacles:
            dist = (obs["center"][0]**2 + obs["center"][1]**2) ** 0.5
            if dist < obs["radius"] + 0.1:  # less than 10cm margin
                return self.motion_planner.plan_stop(emergency=True)
        return None
