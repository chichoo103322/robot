"""ObstacleDetector — Member A

Processes sensor data (depth maps, LIDAR, camera frames) to detect
obstacles in the robot's environment. Outputs a list of detected
obstacle regions for the AvoidancePlanner.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import numpy as np

from ..common.models import SensorData


class ObstacleDetector:
    """Detects obstacles from robot sensor data.

    Uses depth map + LIDAR point cloud fusion for robust detection.
    Configurable detection thresholds and safety margins.

    Detection pipeline:
      1. Depth map → binary obstacle mask (thresholding)
      2. LIDAR points → cluster extraction (DBSCAN-style)
      3. Fusion → final obstacle list with positions and sizes
    """

    # Thresholds
    OBSTACLE_DISTANCE_THRESHOLD = 1.5     # meters — objects closer than this are obstacles
    SAFETY_MARGIN = 0.3                   # meters — extra radius around detected obstacles
    MIN_OBSTACLE_SIZE = 0.05              # meters — ignore very small objects (noise)
    CLUSTER_EPSILON = 0.15                # meters — LIDAR point clustering radius
    MIN_CLUSTER_POINTS = 3                # min points to form an obstacle cluster

    def __init__(self):
        self._lock = threading.Lock()
        self._obstacles: list[dict] = []
        self._last_detection_time: float = 0
        self._on_detection_callbacks: list[Callable[[list[dict]], None]] = []

    # ── Main detection entry ────────────────────────────────────

    def detect(self, sensor_data: SensorData) -> list[dict]:
        """Run full detection pipeline on incoming sensor data.

        Returns list of obstacles: [{"center": (x,y), "radius": r, "confidence": c}, ...]
        """
        obstacles: list[dict] = []

        # Pipeline step 1: depth map obstacles
        if sensor_data.depth_map is not None:
            depth_obstacles = self._detect_from_depth(sensor_data.depth_map)
            obstacles.extend(depth_obstacles)

        # Pipeline step 2: LIDAR point cloud obstacles
        if sensor_data.lidar_points:
            lidar_obstacles = self._detect_from_lidar(sensor_data.lidar_points)
            obstacles.extend(lidar_obstacles)

        # Pipeline step 3: merge overlapping detections
        obstacles = self._merge_obstacles(obstacles)

        # Post-processing: add safety margin
        for obs in obstacles:
            obs["radius"] += self.SAFETY_MARGIN

        with self._lock:
            self._obstacles = obstacles
            self._last_detection_time = time.time()

        # Notify listeners
        for cb in self._on_detection_callbacks:
            try:
                cb(obstacles)
            except Exception:
                pass

        return obstacles

    # ── Detection methods ───────────────────────────────────────

    def _detect_from_depth(self, depth_map: list) -> list[dict]:
        """Extract obstacles from a 2D depth map.

        depth_map: 2D array (rows x cols), each value = distance in meters.
        0 or very large values = no reading.
        """
        try:
            arr = np.array(depth_map, dtype=np.float32)
        except Exception:
            return []

        # Binary mask: pixels closer than threshold are potential obstacles
        valid = (arr > 0) & (arr < self.OBSTACLE_DISTANCE_THRESHOLD)
        if not np.any(valid):
            return []

        # Simple connected-component labeling via flood-fill
        obstacles = self._connected_components(arr, valid)
        return obstacles

    def _detect_from_lidar(self, lidar_points: list[tuple]) -> list[dict]:
        """Cluster LIDAR points to find obstacles.

        Uses a simple distance-based clustering (similar to DBSCAN).
        """
        if len(lidar_points) < self.MIN_CLUSTER_POINTS:
            return []

        points = np.array(lidar_points)  # N x 3 (x, y, z)
        visited = np.zeros(len(points), dtype=bool)
        clusters: list[list[int]] = []

        for i in range(len(points)):
            if visited[i]:
                continue
            # Find neighbors within CLUSTER_EPSILON
            dists = np.linalg.norm(points - points[i], axis=1)
            neighbors = np.where(dists < self.CLUSTER_EPSILON)[0]

            if len(neighbors) < self.MIN_CLUSTER_POINTS:
                visited[i] = True
                continue

            # Expand cluster
            cluster: list[int] = []
            seed = [i]
            while seed:
                idx = seed.pop()
                if visited[idx]:
                    continue
                visited[idx] = True
                cluster.append(idx)
                dists = np.linalg.norm(points - points[idx], axis=1)
                new_neighbors = np.where(dists < self.CLUSTER_EPSILON)[0]
                for n in new_neighbors:
                    if not visited[n] and n not in seed:
                        seed.append(n)

            if len(cluster) >= self.MIN_CLUSTER_POINTS:
                clusters.append(cluster)

        # Convert clusters to obstacle dicts
        obstacles: list[dict] = []
        for cluster in clusters:
            cluster_points = points[cluster]
            center = tuple(np.mean(cluster_points[:, :2], axis=0))
            radius = float(np.max(np.linalg.norm(
                cluster_points[:, :2] - np.array(center[:2]), axis=1)))
            radius = max(radius, self.MIN_OBSTACLE_SIZE)
            obstacles.append({
                "center": (float(center[0]), float(center[1])),
                "radius": radius,
                "confidence": min(1.0, len(cluster) / 10.0),
            })

        return obstacles

    # ── Post-processing ─────────────────────────────────────────

    def _merge_obstacles(self, obstacles: list[dict]) -> list[dict]:
        """Merge overlapping obstacle detections from multiple sensors."""
        if len(obstacles) <= 1:
            return obstacles

        merged: list[dict] = []
        used = [False] * len(obstacles)

        for i, obs_a in enumerate(obstacles):
            if used[i]:
                continue
            group = [obs_a]
            used[i] = True
            for j, obs_b in enumerate(obstacles):
                if used[j]:
                    continue
                dist = self._dist(obs_a["center"], obs_b["center"])
                if dist < (obs_a["radius"] + obs_b["radius"]):
                    group.append(obs_b)
                    used[j] = True

            # Merge group into a single obstacle
            if len(group) == 1:
                merged.append(group[0])
            else:
                cx = sum(o["center"][0] for o in group) / len(group)
                cy = sum(o["center"][1] for o in group) / len(group)
                r = max(o["radius"] for o in group)
                conf = max(o.get("confidence", 0.5) for o in group)
                merged.append({
                    "center": (cx, cy),
                    "radius": r,
                    "confidence": conf,
                })

        return merged

    def _connected_components(self, arr: np.ndarray,
                               mask: np.ndarray) -> list[dict]:
        """Extract connected regions from a binary mask (simple 4-connectivity)."""
        rows, cols = arr.shape
        visited = np.zeros_like(mask, dtype=bool)
        obstacles: list[dict] = []

        for r in range(rows):
            for c in range(cols):
                if not mask[r, c] or visited[r, c]:
                    continue
                # Flood fill
                stack = [(r, c)]
                pixels: list[tuple[int, int]] = []
                while stack:
                    cr, cc = stack.pop()
                    if cr < 0 or cr >= rows or cc < 0 or cc >= cols:
                        continue
                    if visited[cr, cc] or not mask[cr, cc]:
                        continue
                    visited[cr, cc] = True
                    pixels.append((cr, cc))
                    for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                        stack.append((cr + dr, cc + dc))

                if len(pixels) < 5:  # too small — noise
                    continue

                # Calculate obstacle properties
                r_vals = [p[0] for p in pixels]
                c_vals = [p[1] for p in pixels]
                center_r = sum(r_vals) / len(r_vals)
                center_c = sum(c_vals) / len(c_vals)
                # Approximate world coordinates (simplified — real impl needs camera params)
                world_x = (center_c - cols / 2) * 0.01  # rough scaling
                world_y = float(arr[int(center_r), int(center_c)])  # depth = forward distance
                radius = max(
                    (max(r_vals) - min(r_vals)) * 0.005,
                    (max(c_vals) - min(c_vals)) * 0.005,
                    self.MIN_OBSTACLE_SIZE,
                )

                obstacles.append({
                    "center": (world_x, world_y),
                    "radius": radius,
                    "confidence": 0.7,  # depth-based confidence
                })

        return obstacles

    # ── Public helpers ──────────────────────────────────────────

    def get_obstacles(self) -> list[dict]:
        with self._lock:
            return list(self._obstacles)

    def on_detection(self, callback: Callable[[list[dict]], None]) -> None:
        """Register callback invoked on each detection cycle."""
        self._on_detection_callbacks.append(callback)

    def is_path_blocked(self, direction: tuple[float, float],
                         distance: float = 1.0) -> bool:
        """Quick check: is there an obstacle in the given direction within distance?"""
        with self._lock:
            for obs in self._obstacles:
                # Project obstacle center onto direction vector
                cx, cy = obs["center"]
                r = obs["radius"]
                # Check if obstacle intersects a cylinder along the direction
                # Simplified: check if obstacle center is within distance + radius
                if (cx * direction[0] + cy * direction[1]) > 0:
                    dist_to_obs = (cx**2 + cy**2) ** 0.5
                    if dist_to_obs - r < distance:
                        return True
            return False

    @staticmethod
    def _dist(a: tuple, b: tuple) -> float:
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
