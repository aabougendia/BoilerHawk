"""Waypoint survey (area coverage) mission strategy.

Copyright 2026 BoilerHawk — MIT License.

Generates a lawnmower (boustrophedon) pattern over a rectangular area
and flies each waypoint sequentially.
"""

from typing import Any, Dict, Optional

from geometry_msgs.msg import PoseStamped

from mission_manager.strategies.base_strategy import MissionStrategy
from mission_manager.utils.geo_utils import (
    distance_xy,
    generate_lawnmower,
    make_pose,
)


class WaypointSurveyStrategy(MissionStrategy):
    """Fly a lawnmower pattern to cover a rectangular area."""

    name = "waypoint_survey"

    def __init__(self) -> None:
        self._waypoints: list = []
        self._current_idx: int = 0
        self._altitude: float = 2.0

    # ------------------------------------------------------------------ #
    #  Lifecycle
    # ------------------------------------------------------------------ #

    def initialize(self, params: Dict[str, Any]) -> bool:
        """Expected *params* keys.

        Required:
            min_x, min_y, max_x, max_y  — bounding box (metres, local frame)
        Optional:
            altitude   (float, default 2.0)
            spacing    (float, default 1.0 m between rows)
            frame_id   (str,   default "map")
            waypoints  (list of [x, y] — override auto-generated pattern)
        """
        self.reset()

        # Allow raw waypoint list to bypass lawnmower generation
        raw_wps = params.get("waypoints")
        if raw_wps is not None:
            self._altitude = float(params.get("altitude", 2.0))
            frame = params.get("frame_id", "map")
            for wp in raw_wps:
                if len(wp) >= 3:
                    self._waypoints.append(make_pose(wp[0], wp[1], wp[2], frame))
                else:
                    self._waypoints.append(
                        make_pose(wp[0], wp[1], self._altitude, frame)
                    )
            return len(self._waypoints) > 0

        # Auto-generate lawnmower pattern
        try:
            min_x = float(params["min_x"])
            min_y = float(params["min_y"])
            max_x = float(params["max_x"])
            max_y = float(params["max_y"])
        except (KeyError, TypeError, ValueError):
            return False

        self._altitude = float(params.get("altitude", 2.0))
        spacing = float(params.get("spacing", 1.0))
        frame = params.get("frame_id", "map")

        self._waypoints = generate_lawnmower(
            min_x, min_y, max_x, max_y, self._altitude, spacing, frame
        )
        return len(self._waypoints) > 0

    def reset(self) -> None:
        self._waypoints = []
        self._current_idx = 0

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
            return True
        return False

    def is_complete(self) -> bool:
        return self._current_idx >= len(self._waypoints)

    # ------------------------------------------------------------------ #
    #  Reporting
    # ------------------------------------------------------------------ #

    def get_progress(self) -> str:
        total = len(self._waypoints)
        if total == 0:
            return "No waypoints loaded"
        done = min(self._current_idx, total)
        pct = int(done / total * 100)
        return f"Waypoint {done}/{total} ({pct}%)"

    def get_status_dict(self) -> Dict[str, Any]:
        base = super().get_status_dict()
        base["current_wp"] = self._current_idx
        base["total_wps"] = len(self._waypoints)
        return base
