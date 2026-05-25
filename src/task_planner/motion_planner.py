"""MotionPlanner — Member A

Generates Action objects for specific robot motions with appropriate
parameters. Supports all 5+ required actions and can compose them
into multi-action sequences.
"""

from __future__ import annotations

from ..common.enums import ActionType, TaskPriority
from ..common.interfaces import IMotionPlanner
from ..common.models import Action


class MotionPlanner(IMotionPlanner):
    """Plans individual motions and composite action sequences.

    Each method returns a fully-parameterized Action ready for scheduling.
    Action parameters follow the convention expected by the robot controller:

    Walk actions:
      - distance_m: float (meters)
      - speed: float (m/s, default 0.5)

    Turn actions:
      - angle_deg: float (degrees, positive = clockwise)
      - angular_speed: float (deg/s, default 30)

    Avoidance actions:
      - waypoints: list of (x, y) tuples
      - speed: float
    """

    DEFAULT_WALK_SPEED = 0.5        # m/s
    MAX_WALK_SPEED = 1.5
    DEFAULT_ANGULAR_SPEED = 30.0    # deg/s
    MAX_ANGULAR_SPEED = 90.0

    # ── Single actions ──────────────────────────────────────────

    def plan_straight_walk(self, distance_m: float,
                           speed: float = DEFAULT_WALK_SPEED) -> Action:
        """Plan straight line walking."""
        self._validate_speed(speed)
        return Action(
            action_type=ActionType.WALK_STRAIGHT,
            params={
                "distance_m": distance_m,
                "speed": min(speed, self.MAX_WALK_SPEED),
            },
        )

    def plan_backward_walk(self, distance_m: float,
                           speed: float = DEFAULT_WALK_SPEED) -> Action:
        """Plan backward walking."""
        self._validate_speed(speed)
        return Action(
            action_type=ActionType.WALK_BACKWARD,
            params={
                "distance_m": distance_m,
                "speed": min(speed, self.MAX_WALK_SPEED),
            },
        )

    def plan_turn_in_place(self, angle_deg: float,
                           angular_speed: float = DEFAULT_ANGULAR_SPEED) -> Action:
        """Plan turning in place (0~360 degrees)."""
        return Action(
            action_type=ActionType.TURN_IN_PLACE,
            params={
                "angle_deg": self._normalize_angle(angle_deg),
                "angular_speed": min(angular_speed, self.MAX_ANGULAR_SPEED),
            },
        )

    def plan_turn_walk(self, distance_m: float, angle_deg: float,
                       speed: float = DEFAULT_WALK_SPEED) -> Action:
        """Plan walking while turning (curved path)."""
        self._validate_speed(speed)
        return Action(
            action_type=ActionType.TURN_WALK,
            params={
                "distance_m": distance_m,
                "angle_deg": self._normalize_angle(angle_deg),
                "speed": min(speed, self.MAX_WALK_SPEED),
            },
        )

    def plan_stop(self, emergency: bool = False) -> Action:
        """Plan immediate stop."""
        return Action(
            action_type=ActionType.STOP,
            params={"emergency": emergency},
            priority=TaskPriority.EMERGENCY if emergency else TaskPriority.HIGH,
        )

    def plan_sidestep(self, distance_m: float,
                      speed: float = DEFAULT_WALK_SPEED) -> Action:
        """Plan lateral (sideways) movement."""
        self._validate_speed(speed)
        return Action(
            action_type=ActionType.SIDESTEP,
            params={
                "distance_m": distance_m,
                "speed": min(speed, self.MAX_WALK_SPEED),
            },
        )

    # ── Obstacle avoidance ──────────────────────────────────────

    def plan_avoidance_path(self, obstacles: list[dict],
                            current_pos: tuple) -> list[Action]:
        """Generate a sequence of actions to avoid detected obstacles.

        obstacles: list of {"center": (x, y), "radius": float, "type": str}
        current_pos: (x, y) current robot position
        Returns a list of actions that navigate around obstacles.
        """
        if not obstacles:
            return [self.plan_straight_walk(0.5)]

        actions: list[Action] = []
        nearest = min(obstacles, key=lambda o: self._dist(current_pos, o["center"]))

        # Simple strategy: sidestep to clear the obstacle, then resume forward
        clearance = nearest.get("radius", 0.3) + 0.3  # extra margin

        # Determine sidestep direction (left or right)
        obs_x, obs_y = nearest["center"]
        # Default: sidestep right if obstacle is more on the left, else left
        if obs_x < current_pos[0]:
            actions.append(self.plan_sidestep(clearance))
        else:
            actions.append(self.plan_sidestep(-clearance))

        # Move forward past the obstacle
        actions.append(self.plan_straight_walk(clearance * 1.5))

        # Return to original line
        if obs_x < current_pos[0]:
            actions.append(self.plan_sidestep(-clearance))
        else:
            actions.append(self.plan_sidestep(clearance))

        return actions

    # ── Sequence builder ────────────────────────────────────────

    def build_action_sequence(self, descriptions: list[dict]) -> list[Action]:
        """Build actions from high-level descriptions.

        Example:
            [
                {"type": "walk_straight", "distance": 2.0, "speed": 0.5},
                {"type": "turn_in_place", "angle": 90},
                {"type": "walk_straight", "distance": 1.0},
                {"type": "stop"},
            ]
        """
        actions: list[Action] = []
        dispatch = {
            "walk_straight": lambda d: self.plan_straight_walk(
                d.get("distance", 1.0), d.get("speed", self.DEFAULT_WALK_SPEED)),
            "walk_backward": lambda d: self.plan_backward_walk(
                d.get("distance", 1.0), d.get("speed", self.DEFAULT_WALK_SPEED)),
            "turn_in_place": lambda d: self.plan_turn_in_place(
                d.get("angle", 90), d.get("angular_speed", self.DEFAULT_ANGULAR_SPEED)),
            "turn_walk": lambda d: self.plan_turn_walk(
                d.get("distance", 1.0), d.get("angle", 90),
                d.get("speed", self.DEFAULT_WALK_SPEED)),
            "stop": lambda d: self.plan_stop(d.get("emergency", False)),
            "sidestep": lambda d: self.plan_sidestep(
                d.get("distance", 0.5), d.get("speed", self.DEFAULT_WALK_SPEED)),
        }
        for desc in descriptions:
            action_type = desc["type"]
            if action_type in dispatch:
                actions.append(dispatch[action_type](desc))
        return actions

    # ── Helpers ─────────────────────────────────────────────────

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-180, 180]."""
        angle = angle % 360
        if angle > 180:
            angle -= 360
        return angle

    @staticmethod
    def _validate_speed(speed: float) -> None:
        if speed <= 0:
            raise ValueError(f"Speed must be positive, got {speed}")

    @staticmethod
    def _dist(a: tuple, b: tuple) -> float:
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
