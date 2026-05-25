"""LogSystem — Member B

Structured logging for the robot system. Wraps Python's logging
with robot-specific formatters and provides query APIs for the UI.
"""

from __future__ import annotations

import logging
import sys
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Optional


class LogSystem:
    """System-wide logger with file rotation and UI-friendly query methods.

    Usage:
        log = LogSystem(log_dir="./logs")
        log.info("task_planner", "Task started")
        log.error("communication", "Connection lost", exc_info=True)
    """

    LOG_FORMAT = "%(asctime)s | %(levelname)-7s | %(source)-18s | %(message)s"
    DATE_FORMAT = "%Y-%m-%d %H:%M:%S"

    def __init__(self, log_dir: str = "./logs", level: int = logging.DEBUG):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self._logger = logging.getLogger("robot_system")
        self._logger.setLevel(level)
        self._logger.handlers.clear()

        # Console handler
        console = logging.StreamHandler(sys.stdout)
        console.setLevel(logging.INFO)
        console.setFormatter(logging.Formatter(self.LOG_FORMAT, self.DATE_FORMAT))
        self._logger.addHandler(console)

        # File handler (rotating, max 10MB each, keep 5 backups)
        file_handler = RotatingFileHandler(
            self.log_dir / "robot.log",
            maxBytes=10 * 1024 * 1024,
            backupCount=5,
            encoding="utf-8",
        )
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(logging.Formatter(self.LOG_FORMAT, self.DATE_FORMAT))
        self._logger.addHandler(file_handler)

    def _log(self, level: int, source: str, message: str, exc_info: bool = False) -> None:
        extra = {"source": source}
        self._logger.log(level, message, extra=extra, exc_info=exc_info)

    def debug(self, source: str, message: str) -> None:
        self._log(logging.DEBUG, source, message)

    def info(self, source: str, message: str) -> None:
        self._log(logging.INFO, source, message)

    def warning(self, source: str, message: str) -> None:
        self._log(logging.WARNING, source, message)

    def error(self, source: str, message: str, exc_info: bool = False) -> None:
        self._log(logging.ERROR, source, message, exc_info)

    def get_log_file_path(self) -> str:
        return str(self.log_dir / "robot.log")
