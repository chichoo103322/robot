"""RobotDashboard — Member B

Main UI dashboard built with PyQt6. Displays:
  - Robot status panel (state, battery, position, velocity)
  - Action execution progress
  - System log viewer with level filtering
  - Manual control buttons
  - Vision/camera feed area
  - Vision display panel (depth map visualization, obstacle overlay)

Runs as a standalone window. Falls back gracefully if PyQt6 is not installed.
"""

from __future__ import annotations

import sys
import time
from typing import Optional

from ..common.enums import ActionType
from ..common.interfaces import IStatusManager

# Optional PyQt6 import
try:
    from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QLabel, QPushButton, QTextEdit, QGroupBox, QGridLayout,
        QComboBox, QProgressBar, QTabWidget, QSplitter,
    )
    from PyQt6.QtCore import Qt, QTimer
    from PyQt6.QtGui import QFont, QColor, QTextCursor
    HAS_QT = True
except ImportError:
    HAS_QT = False


class RobotDashboard:
    """Main dashboard window for the robot control system.

    Usage:
        dashboard = RobotDashboard(status_manager)
        dashboard.register_control_panel(control_panel)
        dashboard.run()  # blocks until window closes
    """

    def __init__(self, status_manager: IStatusManager):
        self._status_manager = status_manager
        self._control_panel = None
        self._app: Optional[QApplication] = None
        self._window: Optional[QMainWindow] = None
        self._refresh_timer: Optional[QTimer] = None

        # UI widget references
        self._state_label: Optional[QLabel] = None
        self._battery_label: Optional[QLabel] = None
        self._position_label: Optional[QLabel] = None
        self._velocity_label: Optional[QLabel] = None
        self._log_view: Optional[QTextEdit] = None
        self._action_progress: Optional[QProgressBar] = None

        # Vision display
        self._vision_label: Optional[QLabel] = None
        self._vision_display = None

    def register_control_panel(self, control_panel) -> None:
        self._control_panel = control_panel

    def run(self) -> None:
        if not HAS_QT:
            print("[Dashboard] PyQt6 not installed — running in headless mode.")
            print("[Dashboard] Install with: pip install PyQt6")
            # Keep running for CLI interaction
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                pass
            return

        self._app = QApplication(sys.argv)
        self._app.setStyle("Fusion")
        self._window = QMainWindow()
        self._window.setWindowTitle("人形机器人控制系统 — Humanoid Robot Control")
        self._window.resize(1280, 800)

        central = QWidget()
        self._window.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # Top: Status bar
        main_layout.addWidget(self._build_status_bar())

        # Middle: Main content splitter
        splitter = QSplitter(Qt.Orientation.Horizontal)

        # Left: Action controls + progress
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.addWidget(self._build_control_buttons())
        left_layout.addWidget(self._build_action_progress())
        left_layout.addWidget(self._build_manual_actions())
        splitter.addWidget(left_panel)

        # Center: Vision display
        vision_panel = QWidget()
        vision_layout = QVBoxLayout(vision_panel)
        vision_layout.addWidget(self._build_vision_display())
        splitter.addWidget(vision_panel)

        # Right: Log viewer
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.addWidget(self._build_log_viewer())
        splitter.addWidget(right_panel)

        splitter.setSizes([300, 500, 400])
        main_layout.addWidget(splitter)

        # Refresh timer — 100ms = 10Hz UI update
        self._refresh_timer = QTimer()
        self._refresh_timer.timeout.connect(self._refresh_ui)
        self._refresh_timer.start(100)

        # Subscribe to status updates
        self._status_manager.subscribe_status(self._on_status_update)

        self._window.show()
        sys.exit(self._app.exec())

    # ── UI builders ─────────────────────────────────────────────

    def _build_status_bar(self) -> QGroupBox:
        group = QGroupBox("机器人状态 / Robot Status")
        grid = QGridLayout()

        labels = [
            ("状态 / State:", "state"),
            ("电量 / Battery:", "battery"),
            ("位置 / Position:", "position"),
            ("速度 / Velocity:", "velocity"),
        ]

        self._state_label = QLabel("IDLE")
        self._battery_label = QLabel("100%")
        self._position_label = QLabel("(0.00, 0.00, 0.00)")
        self._velocity_label = QLabel("0.00 m/s")

        widgets = [self._state_label, self._battery_label,
                    self._position_label, self._velocity_label]

        for i, (label_text, _) in enumerate(labels):
            grid.addWidget(QLabel(label_text), i // 2, (i % 2) * 2)
            grid.addWidget(widgets[i], i // 2, (i % 2) * 2 + 1)

        group.setLayout(grid)
        return group

    def _build_control_buttons(self) -> QGroupBox:
        group = QGroupBox("任务控制 / Task Control")
        layout = QVBoxLayout()

        btn_start = QPushButton("▶ 开始任务 / Start Task")
        btn_stop = QPushButton("■ 停止任务 / Stop Task")
        btn_pause = QPushButton("⏸ 暂停 / Pause")
        btn_resume = QPushButton("▶ 继续 / Resume")

        btn_start.clicked.connect(lambda: self._on_button("start"))
        btn_stop.clicked.connect(lambda: self._on_button("stop"))
        btn_pause.clicked.connect(lambda: self._on_button("pause"))
        btn_resume.clicked.connect(lambda: self._on_button("resume"))

        for btn in [btn_start, btn_stop, btn_pause, btn_resume]:
            btn.setMinimumHeight(36)
            layout.addWidget(btn)

        group.setLayout(layout)
        return group

    def _build_action_progress(self) -> QGroupBox:
        group = QGroupBox("动作进度 / Action Progress")
        layout = QVBoxLayout()
        self._action_progress = QProgressBar()
        self._action_progress.setRange(0, 100)
        self._action_progress.setValue(0)
        layout.addWidget(self._action_progress)
        group.setLayout(layout)
        return group

    def _build_manual_actions(self) -> QGroupBox:
        group = QGroupBox("手动动作 / Manual Actions")
        layout = QVBoxLayout()

        combo = QComboBox()
        combo.addItems([
            "walk_straight - 直线行走",
            "turn_in_place - 原地掉头",
            "turn_walk - 转弯行走",
            "stop - 停止",
            "walk_backward - 后退",
            "sidestep - 侧向移动",
        ])
        layout.addWidget(combo)

        btn_send = QPushButton("发送 / Send")
        btn_send.clicked.connect(lambda: self._on_manual_action(combo.currentText()))
        layout.addWidget(btn_send)

        group.setLayout(layout)
        return group

    def _build_vision_display(self) -> QGroupBox:
        group = QGroupBox("视觉画面 / Vision Display")
        layout = QVBoxLayout()
        self._vision_label = QLabel("等待视频流... / Waiting for video stream...")
        self._vision_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._vision_label.setMinimumSize(400, 300)
        self._vision_label.setStyleSheet(
            "background-color: #1a1a2e; color: #e0e0e0; border: 1px solid #333;"
        )
        layout.addWidget(self._vision_label)
        group.setLayout(layout)
        return group

    def _build_log_viewer(self) -> QGroupBox:
        group = QGroupBox("系统日志 / System Logs")
        layout = QVBoxLayout()

        # Log level filter
        filter_layout = QHBoxLayout()
        filter_layout.addWidget(QLabel("级别:"))
        log_filter = QComboBox()
        log_filter.addItems(["ALL", "DEBUG", "INFO", "WARNING", "ERROR"])
        filter_layout.addWidget(log_filter)
        filter_layout.addStretch()
        layout.addLayout(filter_layout)

        self._log_view = QTextEdit()
        self._log_view.setReadOnly(True)
        self._log_view.setFont(QFont("Menlo", 10))
        self._log_view.setStyleSheet("background-color: #0d1117; color: #c9d1d9;")
        layout.addWidget(self._log_view)

        group.setLayout(layout)
        return group

    # ── UI refresh ──────────────────────────────────────────────

    def _refresh_ui(self) -> None:
        status = self._status_manager.get_robot_status()
        if self._state_label:
            self._state_label.setText(status.state.value.upper())
            color = {"idle": "#58a6ff", "moving": "#3fb950", "avoiding": "#d29922",
                     "stopped": "#f85149", "error": "#f85149"}.get(status.state.value, "#c9d1d9")
            self._state_label.setStyleSheet(f"color: {color}; font-weight: bold;")

        if self._battery_label:
            self._battery_label.setText(f"{status.battery:.0f}%")

        if self._position_label:
            self._position_label.setText(
                f"({status.position[0]:.2f}, {status.position[1]:.2f}, {status.position[2]:.2f})")

        if self._velocity_label:
            self._velocity_label.setText(f"{status.velocity:.2f} m/s")

        # Refresh logs
        if self._log_view:
            logs = self._status_manager.get_logs(30)
            lines = []
            for log in logs:
                ts = time.strftime("%H:%M:%S", time.localtime(log["timestamp"]))
                lines.append(f"[{ts}] [{log['level']:<5}] [{log['source']}] {log['message']}")
            self._log_view.setPlainText("\n".join(lines))
            # Auto-scroll to bottom
            cursor = self._log_view.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.End)
            self._log_view.setTextCursor(cursor)

    def _on_status_update(self, status) -> None:
        """Called from StatusManager subscriber (non-UI thread safe via Qt timer)."""
        pass  # UI refresh is handled by the timer

    # ── Button handlers ─────────────────────────────────────────

    def _on_button(self, action: str) -> None:
        if not self._control_panel:
            return
        if action == "start":
            self._control_panel.start_task("")
        elif action == "stop":
            self._control_panel.stop_task("")
        elif action == "pause":
            self._control_panel.pause_task("")
        elif action == "resume":
            self._control_panel.resume_task("")

    def _on_manual_action(self, text: str) -> None:
        if not self._control_panel:
            return
        action_type_str = text.split(" - ")[0]
        try:
            action_type = ActionType(action_type_str)
            self._control_panel.send_manual_action(action_type)
        except ValueError:
            pass

    # ── Vision display update ───────────────────────────────────

    def update_vision_frame(self, frame_data) -> None:
        """Update the vision display with a new frame (called externally)."""
        if not HAS_QT or not self._vision_label:
            return
        # Frame rendering handled by VisionDisplay helper
        # This is a placeholder — actual rendering depends on frame format
        pass
