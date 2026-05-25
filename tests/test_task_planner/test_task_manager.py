"""Tests for TaskManager — Member A."""

import pytest
from src.common.enums import ActionType, TaskPriority
from src.common.models import Action
from src.task_planner import TaskManager, MotionPlanner


class TestTaskManager:
    def setup_method(self):
        self.tm = TaskManager()
        self.mp = MotionPlanner()

    def test_create_task(self):
        actions = [
            self.mp.plan_straight_walk(1.0),
            self.mp.plan_turn_in_place(90),
            self.mp.plan_stop(),
        ]
        task = self.tm.create_task("test", actions)
        assert task.name == "test"
        assert len(task.actions) == 3
        assert task.task_id in [t.task_id for t in self.tm.get_all_tasks()]

    def test_task_lifecycle(self):
        task = self.tm.create_task("lifecycle", [self.mp.plan_straight_walk(1.0)])
        assert self.tm.start_task(task.task_id) is True
        assert self.tm.get_current_task().is_running

        assert self.tm.pause_task(task.task_id) is True
        assert not self.tm.get_task(task.task_id).is_running

        assert self.tm.resume_task(task.task_id) is True
        assert self.tm.get_task(task.task_id).is_running

        assert self.tm.stop_task(task.task_id) is True
        assert not self.tm.get_task(task.task_id).is_running

    def test_stop_invalid_task(self):
        assert self.tm.stop_task("nonexistent") is False

    def test_get_all_tasks(self):
        self.tm.create_task("a", [self.mp.plan_stop()])
        self.tm.create_task("b", [self.mp.plan_stop()])
        assert len(self.tm.get_all_tasks()) == 2

    def test_advance_action(self):
        actions = [
            self.mp.plan_straight_walk(1.0),
            self.mp.plan_turn_in_place(90),
            self.mp.plan_stop(),
        ]
        task = self.tm.create_task("advance", actions)
        self.tm.start_task(task.task_id)

        next_a = self.tm.advance_action(task.task_id)
        assert next_a is not None
        assert next_a.action_type == ActionType.TURN_IN_PLACE

        next_a = self.tm.advance_action(task.task_id)
        assert next_a.action_type == ActionType.STOP
