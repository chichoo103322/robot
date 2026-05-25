"""Tests for MotionPlanner — Member A."""

import pytest
from src.common.enums import ActionType
from src.task_planner import MotionPlanner


class TestMotionPlanner:
    def setup_method(self):
        self.mp = MotionPlanner()

    def test_plan_straight_walk(self):
        action = self.mp.plan_straight_walk(2.0, speed=0.5)
        assert action.action_type == ActionType.WALK_STRAIGHT
        assert action.params["distance_m"] == 2.0
        assert action.params["speed"] == 0.5

    def test_plan_turn_in_place(self):
        action = self.mp.plan_turn_in_place(90)
        assert action.action_type == ActionType.TURN_IN_PLACE
        assert action.params["angle_deg"] == 90

    def test_plan_turn_walk(self):
        action = self.mp.plan_turn_walk(1.5, 45)
        assert action.action_type == ActionType.TURN_WALK
        assert action.params["distance_m"] == 1.5
        assert action.params["angle_deg"] == 45

    def test_plan_stop(self):
        action = self.mp.plan_stop()
        assert action.action_type == ActionType.STOP

    def test_plan_emergency_stop(self):
        action = self.mp.plan_stop(emergency=True)
        assert action.action_type == ActionType.STOP
        assert action.params["emergency"] is True

    def test_plan_backward_walk(self):
        action = self.mp.plan_backward_walk(1.0)
        assert action.action_type == ActionType.WALK_BACKWARD
        assert action.params["distance_m"] == 1.0

    def test_plan_sidestep(self):
        action = self.mp.plan_sidestep(0.5)
        assert action.action_type == ActionType.SIDESTEP

    def test_build_action_sequence(self):
        actions = self.mp.build_action_sequence([
            {"type": "walk_straight", "distance": 2.0},
            {"type": "turn_in_place", "angle": 90},
            {"type": "walk_straight", "distance": 1.0},
            {"type": "turn_walk", "distance": 1.5, "angle": 45},
            {"type": "stop"},
        ])
        assert len(actions) == 5
        assert actions[0].action_type == ActionType.WALK_STRAIGHT
        assert actions[1].action_type == ActionType.TURN_IN_PLACE
        assert actions[2].action_type == ActionType.WALK_STRAIGHT
        assert actions[3].action_type == ActionType.TURN_WALK
        assert actions[4].action_type == ActionType.STOP

    def test_speed_limit(self):
        action = self.mp.plan_straight_walk(1.0, speed=999)
        assert action.params["speed"] == self.mp.MAX_WALK_SPEED

    def test_invalid_speed(self):
        with pytest.raises(ValueError):
            self.mp.plan_straight_walk(1.0, speed=-1)

    def test_angle_normalization(self):
        action = self.mp.plan_turn_in_place(370)
        assert action.params["angle_deg"] == 10  # 370 % 360, normalized

    def test_plan_avoidance_path_with_obstacles(self):
        obstacles = [{"center": (0.0, 1.0), "radius": 0.3, "type": "box"}]
        current_pos = (0.0, 0.0)
        actions = self.mp.plan_avoidance_path(obstacles, current_pos)
        assert len(actions) == 3  # sidestep + forward + return sidestep

    def test_plan_avoidance_path_no_obstacles(self):
        actions = self.mp.plan_avoidance_path([], (0.0, 0.0))
        assert len(actions) == 1
        assert actions[0].action_type == ActionType.WALK_STRAIGHT
