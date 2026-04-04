"""Search-and-rescue mission strategy.

Copyright 2026 BoilerHawk — MIT License.

Generates an expanding-square search pattern from a given centre point.
When a human is detected the drone pauses the search, approaches the
detection site, hovers briefly, then resumes the pattern or returns
home when the maximum detection count is reached.
"""

import time
from enum import Enum, auto
from typing import Any, Dict, Optional

from geometry_msgs.msg import PoseStamped

from mission_manager.strategies.base_strategy import MissionStrategy
from mission_manager.utils.geo_utils import (
    distance_xy,
    generate_expanding_square,
    make_pose,
)


class _Phase(Enum):
    SEARCHING = auto()
    APPROACHING = auto()
    HOVERING = auto()


class SearchAndRescueStrategy(MissionStrategy):
    """Expanding-square search with human-detection reaction."""

    name = "search_and_rescue"

    def __init__(self) -> None:
        self._waypoints: list = []
        self._current_idx: int = 0
        self._altitude: float = 2.0
        self._detections: list = []
        self._phase = _Phase.SEARCHING
        self._approach_target: Optional[PoseStamped] = None
        self._hover_start: Optional[float] = None
        self._hover_duration: float = 5.0
        self._max_detections: int = 5

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
            hover_duration  (float, default 5.0 s)
            max_detections  (int,   default 5)
            waypoints       (list of [x,y] — override pattern)
        """
        self.reset()

        self._hover_duration = float(params.get("hover_duration", 5.0))
        self._max_detections = int(params.get("max_detections", 5))

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
        self._phase = _Phase.SEARCHING
        self._approach_target = None
        self._hover_start = None

    # ------------------------------------------------------------------ #
    #  Goal generation
    # ------------------------------------------------------------------ #

    def get_next_goal(
        self, current_pose: Optional[PoseStamped]
    ) -> Optional[PoseStamped]:
        if self._phase in (_Phase.APPROACHING, _Phase.HOVERING):
            return self._approach_target
        # SEARCHING
        if self.is_complete():
            return None
        return self._waypoints[self._current_idx]

    def advance(self, current_pose: PoseStamped, threshold: float) -> bool:
        if self.is_complete():
            return False

        # --- APPROACHING: fly toward detection site ---
        if self._phase == _Phase.APPROACHING:
            if (self._approach_target is not None
                    and distance_xy(current_pose, self._approach_target)
                    < threshold):
                self._phase = _Phase.HOVERING
                self._hover_start = time.monotonic()
                return True
            return False

        # --- HOVERING: dwell over detection site ---
        if self._phase == _Phase.HOVERING:
            if (self._hover_start is not None
                    and (time.monotonic() - self._hover_start)
                    >= self._hover_duration):
                if len(self._detections) >= self._max_detections:
                    self._current_idx = len(self._waypoints)
                else:
                    self._phase = _Phase.SEARCHING
                self._approach_target = None
                self._hover_start = None
                return True
            return False

        # --- SEARCHING: normal waypoint traversal ---
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
        """React to a human detection from the detector node.

        Records the detection and interrupts the search to approach
        the detection site and hover.
        """
        if self._phase in (_Phase.APPROACHING, _Phase.HOVERING):
            return

        self._detections.append({
            "pose": detection_pose,
            "label": label,
            "at_wp": self._current_idx,
        })

        p = detection_pose.pose.position
        self._approach_target = make_pose(
            p.x, p.y, self._altitude,
            detection_pose.header.frame_id or "map")
        self._phase = _Phase.APPROACHING

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
        phase = self._phase.name.lower()
        return f"Search {done}/{total} ({pct}%), detections: {det}, {phase}"

    def get_status_dict(self) -> Dict[str, Any]:
        base = super().get_status_dict()
        base["current_wp"] = self._current_idx
        base["total_wps"] = len(self._waypoints)
        base["detections"] = len(self._detections)
        base["phase"] = self._phase.name
        return base
