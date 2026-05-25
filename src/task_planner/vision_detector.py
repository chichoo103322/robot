"""VisionDetector — Member A

Camera-based obstacle detection using OpenCV. Supports:
  - Simulated mode: generates virtual obstacles for testing
  - Depth mode: processes depth maps for obstacle segmentation
  - YOLO mode: uses YOLOv8 for real-time object detection (optional)

The detector outputs obstacle regions that feed into ReactiveAvoidance.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import numpy as np


class VisionDetector:
    """Real-time vision-based obstacle detector.

    Operates in three modes:
      - "simulated": Fake obstacles for dev/testing (no camera needed)
      - "depth": Process depth maps for foreground segmentation
      - "yolo": Use YOLOv8 model for object detection (requires ultralytics)

    Usage:
        detector = VisionDetector(mode="simulated")
        detector.start()
        obs = detector.detect(frame)  # returns list of obstacle dicts
    """

    # Detection parameters
    DANGER_ZONE_NEAR = 0.3    # meters — emergency stop
    DANGER_ZONE_FAR = 1.5     # meters — trigger avoidance
    OBSTACLE_MIN_WIDTH = 0.1  # meters — filter noise

    def __init__(self, mode: str = "simulated",
                 yolo_model: str = "yolov8n.pt",
                 camera_matrix: Optional[np.ndarray] = None):
        self.mode = mode
        self._lock = threading.Lock()
        self._obstacles: list[dict] = []
        self._yolo_model = None
        self._on_detect: list[Callable[[list[dict]], None]] = []

        # Simulated obstacles for demo mode
        self._sim_obstacles: list[dict] = []
        self._sim_enabled = (mode == "simulated")

        if mode == "yolo":
            self._init_yolo(yolo_model)
        if mode == "depth" and camera_matrix is None:
            # Default pinhole model (will be approximate)
            self._camera_matrix = np.array([
                [640, 0, 320],
                [0, 640, 240],
                [0, 0, 1],
            ], dtype=np.float32)
        else:
            self._camera_matrix = camera_matrix

    # ── Public API ───────────────────────────────────────────────

    def detect(self, frame: Optional[np.ndarray] = None,
               depth_map: Optional[np.ndarray] = None) -> list[dict]:
        """Run detection on current frame. Returns list of obstacle dicts.

        Each obstacle: {"center": (x_m, y_m), "radius": r_m,
                        "confidence": 0-1, "label": str}
        Coordinates are in robot-centric frame: x=right, y=forward.
        """
        if self._sim_enabled:
            obstacles = list(self._sim_obstacles)
        elif self.mode == "yolo" and frame is not None:
            obstacles = self._detect_yolo(frame)
        elif self.mode == "depth" and depth_map is not None:
            obstacles = self._detect_depth(depth_map)
        else:
            obstacles = []

        with self._lock:
            self._obstacles = obstacles

        for cb in self._on_detect:
            try:
                cb(obstacles)
            except Exception:
                pass

        return obstacles

    def get_obstacles(self) -> list[dict]:
        with self._lock:
            return list(self._obstacles)

    def on_detect(self, callback: Callable[[list[dict]], None]) -> None:
        self._on_detect.append(callback)

    # ── Simulated obstacles for testing ──────────────────────────

    def add_simulated_obstacle(self, x: float, y: float, radius: float = 0.3,
                                label: str = "obstacle") -> None:
        """Add a simulated obstacle at (x, y) in robot frame."""
        self._sim_obstacles.append({
            "center": (x, y), "radius": radius,
            "confidence": 1.0, "label": label,
        })

    def clear_simulated(self) -> None:
        self._sim_obstacles.clear()

    def is_path_clear(self, target_distance: float,
                       current_pos: tuple = (0, 0),
                       direction: tuple = (0, 1)) -> bool:
        """Check if the forward path is clear for target_distance meters."""
        obstacles = self.get_obstacles()
        for obs in obstacles:
            ox, oy = obs["center"]
            r = obs["radius"]
            # Check if obstacle intersects forward corridor
            # Corridor width = robot_width (≈0.4m) + margin
            corridor_half_width = 0.3
            if abs(ox) < corridor_half_width + r:
                if 0 < oy < target_distance + r:
                    return False
        return True

    def get_nearest_blocking_obstacle(self, target_distance: float = 2.0,
                                       current_pos: tuple = (0, 0)) -> Optional[dict]:
        """Get the nearest obstacle blocking the forward path."""
        obstacles = self.get_obstacles()
        nearest = None
        nearest_dist = float("inf")
        for obs in obstacles:
            ox, oy = obs["center"]
            r = obs["radius"]
            dist = (ox**2 + oy**2) ** 0.5
            if dist < nearest_dist and oy > 0 and oy < target_distance + r:
                if abs(ox) < 0.3 + r:
                    nearest = obs
                    nearest_dist = dist
        return nearest

    # ── Depth-based detection ────────────────────────────────────

    def _detect_depth(self, depth_map: np.ndarray) -> list[dict]:
        """Segment obstacles from a depth image.

        depth_map: HxW float array, values in meters. 0 = no reading.
        """
        if depth_map is None or not np.any(depth_map > 0):
            return []

        # Threshold: pixels closer than DANGER_ZONE_FAR are potential obstacles
        mask = (depth_map > 0) & (depth_map < self.DANGER_ZONE_FAR)
        if not np.any(mask):
            return []

        # Find connected regions
        obstacles = self._find_regions(depth_map, mask)
        return obstacles

    def _find_regions(self, depth_map: np.ndarray,
                       mask: np.ndarray) -> list[dict]:
        """Simple connected-component analysis to find obstacle blobs."""
        from collections import deque

        h, w = mask.shape
        visited = np.zeros_like(mask, dtype=bool)
        obstacles = []

        for y in range(0, h, 4):  # stride for speed
            for x in range(0, w, 4):
                if not mask[y, x] or visited[y, x]:
                    continue

                # BFS to grow region
                region_pixels = []
                q = deque([(y, x)])
                while q:
                    cy, cx = q.popleft()
                    if cy < 0 or cy >= h or cx < 0 or cx >= w:
                        continue
                    if visited[cy, cx] or not mask[cy, cx]:
                        continue
                    visited[cy, cx] = True
                    region_pixels.append((cy, cx))
                    for dy, dx in [(0, 2), (0, -2), (2, 0), (-2, 0)]:
                        ny, nx = cy + dy, cx + dx
                        if 0 <= ny < h and 0 <= nx < w:
                            q.append((ny, nx))

                if len(region_pixels) < 20:  # too small
                    continue

                # Convert to robot-frame coordinates (approximate)
                avg_y = sum(p[0] for p in region_pixels) / len(region_pixels)
                avg_x = sum(p[1] for p in region_pixels) / len(region_pixels)
                depth_val = float(depth_map[int(avg_y), int(avg_x)])

                # Simple pinhole inverse projection
                cx_cam = w / 2
                # Horizontal: angle from optical center
                world_x = (avg_x - cx_cam) * depth_val / (w / 2)  # rough
                world_y = depth_val  # forward distance

                # Radius from region extent
                ys = [p[0] for p in region_pixels]
                xs = [p[1] for p in region_pixels]
                radius = max(
                    (max(ys) - min(ys)) * 0.002,  # ~2mm/pixel rough
                    (max(xs) - min(xs)) * 0.002,
                    0.1,
                )

                obstacles.append({
                    "center": (float(world_x), float(world_y)),
                    "radius": float(radius),
                    "confidence": min(1.0, len(region_pixels) / 200),
                    "label": "obstacle",
                })

        return obstacles

    # ── YOLO detection ───────────────────────────────────────────

    def _init_yolo(self, model_path: str) -> None:
        """Initialize YOLO model (lazy load to avoid import overhead)."""
        try:
            from ultralytics import YOLO
            self._yolo_model = YOLO(model_path)
            print(f"[VisionDetector] YOLO model loaded: {model_path}")
        except ImportError:
            print("[VisionDetector] ultralytics not installed. "
                  "Run: pip install ultralytics. Falling back to simulated mode.")
            self._sim_enabled = True
        except Exception as e:
            print(f"[VisionDetector] Failed to load YOLO model: {e}")
            self._sim_enabled = True

    def _detect_yolo(self, frame: np.ndarray) -> list[dict]:
        """Run YOLO on a camera frame, return obstacle list."""
        if self._yolo_model is None:
            return self._detect_depth(frame)  # fallback: treat as depth

        results = self._yolo_model(frame, verbose=False)
        obstacles = []

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                cls_id = int(box.cls[0])
                label = result.names.get(cls_id, "unknown")
                conf = float(box.conf[0])

                # Filter: keep objects that are obstacles
                obstacle_classes = {
                    "person", "chair", "couch", "table", "box",
                    "bottle", "backpack", "suitcase", "tv", "plant",
                    "wall", "door", "refrigerator", "bookcase",
                }
                # In COCO, common obstacle-like classes
                coco_obstacles = {0, 56, 57, 58, 59, 60, 61, 62, 63,
                                   67, 70, 72, 73, 74, 75}
                if label.lower() in obstacle_classes or cls_id in coco_obstacles:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2
                    w = x2 - x1
                    h = y2 - y1

                    # Approximate world coordinates (needs camera calibration
                    # for accurate conversion; these are relative)
                    h_img, w_img = frame.shape[:2]
                    world_x = (cx - w_img / 2) / (w_img / 2)  # normalized
                    world_y = 1.0 - (cy / h_img)  # top=far, bottom=near
                    radius = max(w, h) / w_img * 1.5  # rough

                    obstacles.append({
                        "center": (float(world_x), float(world_y * self.DANGER_ZONE_FAR)),
                        "radius": float(radius),
                        "confidence": float(conf),
                        "label": label,
                    })

        return obstacles
