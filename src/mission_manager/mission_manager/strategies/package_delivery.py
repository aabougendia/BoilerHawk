"""Package delivery mission strategy.

Copyright 2026 BoilerHawk — MIT License.

Multi-phase strategy: fly to pickup location, attach payload, fly to
dropoff location, detach payload, then complete.  Publishes delivery
commands (attach / detach) on ``/delivery/command`` for the delivery
manager node to relay to Gazebo.
"""

import time
from enum import Enum, auto
from typing import Any, Dict, Optional

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from mission_manager.strategies.base_strategy import MissionStrategy
from mission_manager.utils.geo_utils import distance_xy, make_pose


class _Phase(Enum):
    GOTO_PICKUP = auto()
    PICKUP = auto()
    GOTO_DROPOFF = auto()
    DESCEND = auto()
    DROPOFF = auto()
    COMPLETE = auto()


class PackageDeliveryStrategy(MissionStrategy):
    """Pick up a package, fly to destination, drop it off."""

    name = "package_delivery"

    def __init__(self) -> None:
        self._phase = _Phase.GOTO_PICKUP
        self._pickup: Optional[PoseStamped] = None
        self._dropoff: Optional[PoseStamped] = None
        self._dropoff_low: Optional[PoseStamped] = None
        self._altitude: float = 3.0
        self._drop_altitude: float = 0.5
        self._dwell_time: float = 3.0  # seconds to hover at pickup/dropoff
        self._dwell_start: Optional[float] = None

        # Populated by the mission manager node after construction
        self._delivery_pub = None  # type: ignore[assignment]

    # ------------------------------------------------------------------ #
    #  Lifecycle
    # ------------------------------------------------------------------ #

    def initialize(self, params: Dict[str, Any]) -> bool:
        """Expected *params* keys.

        Required:
            pickup_x, pickup_y   — pickup location (metres, local frame)
            dropoff_x, dropoff_y — dropoff location
        Optional:
            altitude   (float, default 3.0)
            dwell_time (float, default 3.0)  — seconds to hover at each site
            frame_id   (str,   default "map")
        """
        self.reset()
        try:
            px = float(params["pickup_x"])
            py = float(params["pickup_y"])
            dx = float(params["dropoff_x"])
            dy = float(params["dropoff_y"])
        except (KeyError, TypeError, ValueError):
            return False

        self._altitude = float(params.get("altitude", 3.0))
        self._dwell_time = float(params.get("dwell_time", 3.0))
        frame = params.get("frame_id", "map")

        self._pickup = make_pose(px, py, self._altitude, frame)
        self._dropoff = make_pose(dx, dy, self._altitude, frame)
        self._dropoff_low = make_pose(dx, dy, self._drop_altitude, frame)
        return True

    def reset(self) -> None:
        self._phase = _Phase.GOTO_PICKUP
        self._pickup = None
        self._dropoff = None
        self._dropoff_low = None
        self._dwell_start = None

    # ------------------------------------------------------------------ #
    #  Delivery command helper
    # ------------------------------------------------------------------ #

    def set_delivery_publisher(self, pub) -> None:
        """Inject the ROS publisher for ``/delivery/command``."""
        self._delivery_pub = pub

    def _publish_command(self, cmd: str) -> None:
        if self._delivery_pub is not None:
            msg = String()
            msg.data = cmd
            self._delivery_pub.publish(msg)

    # ------------------------------------------------------------------ #
    #  Goal generation
    # ------------------------------------------------------------------ #

    def get_next_goal(
        self, current_pose: Optional[PoseStamped]
    ) -> Optional[PoseStamped]:
        if self._phase == _Phase.GOTO_PICKUP:
            return self._pickup
        if self._phase == _Phase.PICKUP:
            return self._pickup  # hold position
        if self._phase == _Phase.GOTO_DROPOFF:
            return self._dropoff
        if self._phase == _Phase.DESCEND:
            return self._dropoff_low  # hold XY at low altitude
        if self._phase == _Phase.DROPOFF:
            return self._dropoff_low  # hold position
        return None  # COMPLETE

    def advance(self, current_pose: PoseStamped, threshold: float) -> bool:
        if self._phase == _Phase.COMPLETE:
            return False

        # --- GOTO_PICKUP → PICKUP ---
        if self._phase == _Phase.GOTO_PICKUP:
            if self._pickup and distance_xy(current_pose, self._pickup) < threshold:
                self._phase = _Phase.PICKUP
                self._dwell_start = time.monotonic()
                self._publish_command("attach")
                return True
            return False

        # --- PICKUP (dwell) → GOTO_DROPOFF ---
        if self._phase == _Phase.PICKUP:
            if self._dwell_start and (time.monotonic() - self._dwell_start >= self._dwell_time):
                self._phase = _Phase.GOTO_DROPOFF
                self._dwell_start = None
                return True
            return False

        # --- GOTO_DROPOFF → DESCEND ---
        if self._phase == _Phase.GOTO_DROPOFF:
            if self._dropoff and distance_xy(current_pose, self._dropoff) < threshold:
                self._phase = _Phase.DESCEND
                return True
            return False

        # --- DESCEND → DROPOFF (when altitude is low enough) ---
        if self._phase == _Phase.DESCEND:
            cur_z = current_pose.pose.position.z
            if cur_z < self._drop_altitude + 0.3:
                self._phase = _Phase.DROPOFF
                self._dwell_start = time.monotonic()
                self._publish_command("detach")
                return True
            return False

        # --- DROPOFF (dwell) → COMPLETE ---
        if self._phase == _Phase.DROPOFF:
            if self._dwell_start and (time.monotonic() - self._dwell_start >= self._dwell_time):
                self._phase = _Phase.COMPLETE
                self._dwell_start = None
                return True
            return False

        return False

    def is_complete(self) -> bool:
        return self._phase == _Phase.COMPLETE

    def should_rtl_on_complete(self) -> bool:
        """Delivery missions land at the dropoff — no RTL."""
        return False

    def get_landing_position(self) -> Optional[PoseStamped]:
        """Land at the dropoff site."""
        return self._dropoff_low

    # ------------------------------------------------------------------ #
    #  Reporting
    # ------------------------------------------------------------------ #

    def get_progress(self) -> str:
        phase_labels = {
            _Phase.GOTO_PICKUP: "Flying to pickup",
            _Phase.PICKUP: "Picking up package",
            _Phase.GOTO_DROPOFF: "Flying to dropoff",
            _Phase.DESCEND: "Descending to drop altitude",
            _Phase.DROPOFF: "Dropping off package",
            _Phase.COMPLETE: "Delivery complete",
        }
        return phase_labels.get(self._phase, "Unknown phase")

    def get_status_dict(self) -> Dict[str, Any]:
        base = super().get_status_dict()
        base["phase"] = self._phase.name
        if self._pickup:
            base["pickup_x"] = self._pickup.pose.position.x
            base["pickup_y"] = self._pickup.pose.position.y
        if self._dropoff:
            base["dropoff_x"] = self._dropoff.pose.position.x
            base["dropoff_y"] = self._dropoff.pose.position.y
        return base
