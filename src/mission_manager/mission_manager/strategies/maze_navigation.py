"""Maze navigation mission strategy.

Copyright 2026 BoilerHawk — MIT License.

Single-goal strategy that publishes one destination and relies on
the planning node's A* planner to find an obstacle-free path through
the maze.  This mirrors the current hardcoded pipeline but allows
the goal to be set dynamically via the mission manager.
"""

from typing import Any, Dict, Optional

from geometry_msgs.msg import PoseStamped

from mission_manager.strategies.base_strategy import MissionStrategy
from mission_manager.utils.geo_utils import distance_xy, make_pose


class MazeNavigationStrategy(MissionStrategy):
    """Navigate through a maze to a single goal position."""

    name = "maze_navigation"

    def __init__(self) -> None:
        self._goal: Optional[PoseStamped] = None
        self._complete: bool = False
        self._altitude: float = 2.0

    # ------------------------------------------------------------------ #
    #  Lifecycle
    # ------------------------------------------------------------------ #

    def initialize(self, params: Dict[str, Any]) -> bool:
        """Expected *params* keys.

        Required:
            goal_x, goal_y — target position (metres, local frame)
        Optional:
            altitude   (float, default 2.0)
            frame_id   (str,   default "map")
        """
        self.reset()
        try:
            gx = float(params["goal_x"])
            gy = float(params["goal_y"])
        except (KeyError, TypeError, ValueError):
            return False

        self._altitude = float(params.get("altitude", 2.0))
        frame = params.get("frame_id", "map")
        self._goal = make_pose(gx, gy, self._altitude, frame)
        return True

    def reset(self) -> None:
        self._goal = None
        self._complete = False

    # ------------------------------------------------------------------ #
    #  Goal generation
    # ------------------------------------------------------------------ #

    def get_next_goal(
        self, current_pose: Optional[PoseStamped]
    ) -> Optional[PoseStamped]:
        if self._complete:
            return None
        return self._goal

    def advance(self, current_pose: PoseStamped, threshold: float) -> bool:
        if self._complete or self._goal is None:
            return False
        if distance_xy(current_pose, self._goal) < threshold:
            self._complete = True
            return True
        return False

    def is_complete(self) -> bool:
        return self._complete

    # ------------------------------------------------------------------ #
    #  Reporting
    # ------------------------------------------------------------------ #

    def get_progress(self) -> str:
        if self._goal is None:
            return "No goal set"
        if self._complete:
            return "Maze navigation complete"
        gx = self._goal.pose.position.x
        gy = self._goal.pose.position.y
        return f"Navigating to ({gx:.1f}, {gy:.1f})"

    def get_status_dict(self) -> Dict[str, Any]:
        base = super().get_status_dict()
        if self._goal is not None:
            base["goal_x"] = self._goal.pose.position.x
            base["goal_y"] = self._goal.pose.position.y
        return base
