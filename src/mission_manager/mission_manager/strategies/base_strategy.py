"""Abstract base class for mission strategies.

Copyright 2026 BoilerHawk — MIT License.

Every concrete mission type (survey, patrol, search-and-rescue, …)
must subclass ``MissionStrategy`` and implement the abstract methods.
The mission manager node calls these methods during the EXECUTING state
to obtain successive goal poses that are forwarded to the planning layer.
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional

from geometry_msgs.msg import PoseStamped


class MissionStrategy(ABC):
    """Interface that all mission strategies must implement."""

    # Human-readable name (override in subclasses)
    name: str = "base"

    # ------------------------------------------------------------------ #
    #  Lifecycle
    # ------------------------------------------------------------------ #

    @abstractmethod
    def initialize(self, params: Dict[str, Any]) -> bool:
        """Set up the strategy with *params* loaded from the service call.

        Returns ``True`` on success, ``False`` if params are invalid.
        """

    @abstractmethod
    def reset(self) -> None:
        """Reset internal state so the strategy can be re-used."""

    # ------------------------------------------------------------------ #
    #  Goal generation
    # ------------------------------------------------------------------ #

    @abstractmethod
    def get_next_goal(
        self, current_pose: Optional[PoseStamped]
    ) -> Optional[PoseStamped]:
        """Return the next goal the drone should fly to.

        Called at the FSM tick rate while in EXECUTING state.
        Return ``None`` when there is no new goal (hold position).
        *current_pose* may be ``None`` if the drone pose is unknown yet.
        """

    @abstractmethod
    def advance(self, current_pose: PoseStamped, threshold: float) -> bool:
        """Check if the drone has reached the current goal and advance.

        Returns ``True`` if a new goal has been activated (i.e. the drone
        reached the previous one within *threshold* metres).
        """

    @abstractmethod
    def is_complete(self) -> bool:
        """Return ``True`` when the mission has no remaining goals."""

    # ------------------------------------------------------------------ #
    #  Pause / resume
    # ------------------------------------------------------------------ #

    def on_pause(self) -> None:
        """Called when the mission is paused.  Override if needed."""

    def on_resume(self) -> None:
        """Called when the mission is resumed.  Override if needed."""

    # ------------------------------------------------------------------ #
    #  Reporting
    # ------------------------------------------------------------------ #

    @abstractmethod
    def get_progress(self) -> str:
        """Return a human-readable progress string."""

    def get_status_dict(self) -> Dict[str, Any]:
        """Return machine-readable status (override for richer telemetry)."""
        return {
            "strategy": self.name,
            "progress": self.get_progress(),
            "complete": self.is_complete(),
        }
