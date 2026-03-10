"""Perimeter patrol mission strategy.

Copyright 2026 BoilerHawk — MIT License.

Repeatedly circuits a list of perimeter waypoints for a given number of
loops (or indefinitely).
"""

from typing import Any, Dict, List, Optional, Tuple

from geometry_msgs.msg import PoseStamped

from mission_manager.strategies.base_strategy import MissionStrategy
from mission_manager.utils.geo_utils import (
    distance_xy,
    generate_perimeter,
    make_pose,
)


class PerimeterPatrolStrategy(MissionStrategy):
    """Repeated circuit patrol along a polygon perimeter."""

    name = "perimeter_patrol"

    def __init__(self) -> None:
        self._waypoints: list = []
        self._current_idx: int = 0
        self._current_loop: int = 0
        self._max_loops: int = 1  # 0 = infinite
        self._altitude: float = 2.0

    # ------------------------------------------------------------------ #
    #  Lifecycle
    # ------------------------------------------------------------------ #

    def initialize(self, params: Dict[str, Any]) -> bool:
        """Expected *params* keys.

        Required:
            vertices — list of [x, y] perimeter points
        Optional:
            altitude  (float, default 2.0)
            loops     (int,   default 1; 0 = infinite)
            frame_id  (str,   default "map")
        """
        self.reset()

        raw_verts: Optional[List[Tuple[float, float]]] = params.get("vertices")
        if raw_verts is None or len(raw_verts) < 2:
            return False

        self._altitude = float(params.get("altitude", 2.0))
        self._max_loops = int(params.get("loops", 1))
        frame = params.get("frame_id", "map")

        self._waypoints = generate_perimeter(
            [(float(v[0]), float(v[1])) for v in raw_verts],
            self._altitude,
            frame,
        )
        return len(self._waypoints) >= 2

    def reset(self) -> None:
        self._waypoints = []
        self._current_idx = 0
        self._current_loop = 0

    # ------------------------------------------------------------------ #
    #  Goal generation
    # ------------------------------------------------------------------ #

    def get_next_goal(
        self, current_pose: Optional[PoseStamped]
    ) -> Optional[PoseStamped]:
        if self.is_complete():
            return None
        return self._waypoints[self._current_idx]

    def advance(self, current_pose: PoseStamped, threshold: float) -> bool:
        if self.is_complete():
            return False

        goal = self._waypoints[self._current_idx]
        if distance_xy(current_pose, goal) < threshold:
            self._current_idx += 1
            # Wrap around at end of perimeter
            if self._current_idx >= len(self._waypoints):
                self._current_idx = 0
                self._current_loop += 1
            return True
        return False

    def is_complete(self) -> bool:
        if self._max_loops == 0:
            # Infinite patrol — never complete
            return False
        return self._current_loop >= self._max_loops

    # ------------------------------------------------------------------ #
    #  Reporting
    # ------------------------------------------------------------------ #

    def get_progress(self) -> str:
        total_wps = len(self._waypoints)
        if total_wps == 0:
            return "No patrol route loaded"
        loop_str = (
            f"Loop {self._current_loop + 1}/{self._max_loops}"
            if self._max_loops > 0
            else f"Loop {self._current_loop + 1}/∞"
        )
        return f"{loop_str}, waypoint {self._current_idx + 1}/{total_wps}"

    def get_status_dict(self) -> Dict[str, Any]:
        base = super().get_status_dict()
        base["current_wp"] = self._current_idx
        base["total_wps"] = len(self._waypoints)
        base["current_loop"] = self._current_loop
        base["max_loops"] = self._max_loops
        return base
