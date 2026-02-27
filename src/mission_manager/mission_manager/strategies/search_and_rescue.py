"""Search-and-rescue mission strategy.

Copyright 2026 BoilerHawk — MIT License.

Generates an expanding-square search pattern from a given centre point.
Future versions can hook into a perception callback to react to detections.
"""

from typing import Any, Dict, Optional

from geometry_msgs.msg import PoseStamped

from mission_manager.strategies.base_strategy import MissionStrategy
from mission_manager.utils.geo_utils import (
    distance_xy,
    generate_expanding_square,
    make_pose,
)


class SearchAndRescueStrategy(MissionStrategy):
    """Expanding-square search pattern for search-and-rescue missions."""

    name = "search_and_rescue"

    def __init__(self) -> None:
        self._waypoints: list = []
        self._current_idx: int = 0
        self._altitude: float = 2.0
        self._detections: list = []

    # ------------------------------------------------------------------ #
    #  Lifecycle
    # ------------------------------------------------------------------ #

    def initialize(self, params: Dict[str, Any]) -> bool:
        """Expected *params* keys.

        Required:
            center_x, center_y — search centre (metres, local frame)
        Optional:
            altitude        (float, default 2.0)
            initial_leg     (float, default 2.0 m)
            leg_increment   (float, default 2.0 m)
            num_legs        (int,   default 16)
            frame_id        (str,   default "map")
            waypoints       (list of [x,y] — override pattern)
        """
        self.reset()

        # Allow raw waypoint override
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

        # Auto-generate expanding square
        try:
            cx = float(params["center_x"])
            cy = float(params["center_y"])
        except (KeyError, TypeError, ValueError):
            return False

        self._altitude = float(params.get("altitude", 2.0))
        initial_leg = float(params.get("initial_leg", 2.0))
        increment = float(params.get("leg_increment", 2.0))
        num_legs = int(params.get("num_legs", 16))
        frame = params.get("frame_id", "map")

        self._waypoints = generate_expanding_square(
            cx, cy, self._altitude, initial_leg, increment, num_legs, frame
        )
        return len(self._waypoints) > 0

    def reset(self) -> None:
        self._waypoints = []
        self._current_idx = 0
        self._detections = []

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
    #  Detection hook (future use)
    # ------------------------------------------------------------------ #

    def on_detection(self, detection_pose: PoseStamped, label: str = "") -> None:
        """Record a detection from perception.

        Future: could pause mission, loiter, or mark and continue.
        """
        self._detections.append({
            "pose": detection_pose,
            "label": label,
            "at_wp": self._current_idx,
        })

    # ------------------------------------------------------------------ #
    #  Reporting
    # ------------------------------------------------------------------ #

    def get_progress(self) -> str:
        total = len(self._waypoints)
        if total == 0:
            return "No search pattern loaded"
        done = min(self._current_idx, total)
        pct = int(done / total * 100)
        det = len(self._detections)
        return f"Search {done}/{total} ({pct}%), detections: {det}"

    def get_status_dict(self) -> Dict[str, Any]:
        base = super().get_status_dict()
        base["current_wp"] = self._current_idx
        base["total_wps"] = len(self._waypoints)
        base["detections"] = len(self._detections)
        return base
