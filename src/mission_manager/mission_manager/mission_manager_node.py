"""Mission Manager Node — FSM-based application-layer orchestrator.

Copyright 2026 BoilerHawk — MIT License.

Sits above the sensor → perception → planning → control pipeline and
provides high-level mission lifecycle management (load, start, pause,
resume, abort, RTL) via ROS 2 services.

The node is intentionally **generic**: it does not depend on any specific
downstream node being alive.  When the planning pipeline is unavailable
the manager publishes goals on ``/mission/goal`` and reports progress;
it is the responsibility of the planning / control stack to consume
those goals when ready.
"""

import json
import logging
import os
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool

from mission_manager.mission_state import (
    MissionState,
    is_valid_transition,
    get_valid_transitions,
)
from mission_manager.strategies.base_strategy import MissionStrategy
from mission_manager.strategies.waypoint_survey import WaypointSurveyStrategy
from mission_manager.strategies.search_and_rescue import SearchAndRescueStrategy
from mission_manager.strategies.perimeter_patrol import PerimeterPatrolStrategy
from mission_manager.strategies.maze_navigation import MazeNavigationStrategy
from mission_manager.strategies.package_delivery import PackageDeliveryStrategy
from mission_manager.utils.geo_utils import make_pose, distance_xy


# ------------------------------------------------------------------ #
#  Strategy registry — add new strategies here
# ------------------------------------------------------------------ #
STRATEGY_REGISTRY: Dict[str, type] = {
    "waypoint_survey": WaypointSurveyStrategy,
    "search_and_rescue": SearchAndRescueStrategy,
    "perimeter_patrol": PerimeterPatrolStrategy,
    "maze_navigation": MazeNavigationStrategy,
    "package_delivery": PackageDeliveryStrategy,
}


class MissionManagerNode(Node):
    """ROS 2 node that orchestrates autonomous missions via an FSM."""

    def __init__(self) -> None:
        super().__init__("mission_manager_node")

        # File logger for debugging
        log_dir = os.path.expanduser('~/BoilerHawk/BoilerHawk/logs')
        os.makedirs(log_dir, exist_ok=True)
        self.flog = logging.getLogger('bhawk.mission_manager')
        self.flog.setLevel(logging.DEBUG)
        if not self.flog.handlers:
            fh = logging.FileHandler(
                os.path.join(log_dir, 'mission_manager.log'), mode='w')
            fh.setFormatter(logging.Formatter(
                '%(asctime)s [%(levelname)s] %(message)s',
                datefmt='%H:%M:%S'))
            self.flog.addHandler(fh)
        self.flog.info('=== Mission Manager node started ===')

        # ------------------------------------------------------------ #
        #  Parameters
        # ------------------------------------------------------------ #
        self.declare_parameter("takeoff_altitude", 2.0)
        self.declare_parameter("waypoint_threshold", 0.5)
        self.declare_parameter("fsm_tick_rate", 2.0)
        self.declare_parameter("goal_publish_rate", 1.0)
        self.declare_parameter("health_check_rate", 0.5)
        self.declare_parameter("health_timeout", 5.0)
        self.declare_parameter("rtl_x", 0.0)
        self.declare_parameter("rtl_y", 0.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("strategy_name", "waypoint_survey")
        self.declare_parameter("strategy_params", "{}")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("auto_start_delay", 10.0)

        self._takeoff_alt = self.get_parameter("takeoff_altitude").value
        self._wp_threshold = self.get_parameter("waypoint_threshold").value
        self._health_timeout = self.get_parameter("health_timeout").value
        self._rtl_x = self.get_parameter("rtl_x").value
        self._rtl_y = self.get_parameter("rtl_y").value
        self._frame_id = self.get_parameter("frame_id").value

        # ------------------------------------------------------------ #
        #  FSM state
        # ------------------------------------------------------------ #
        self._state = MissionState.IDLE
        self._strategy: Optional[MissionStrategy] = None
        self._current_pose: Optional[PoseStamped] = None
        self._current_goal: Optional[PoseStamped] = None
        self._landing_position: Optional[PoseStamped] = None

        # Health-check timestamps (seconds, monotonic)
        self._last_control_stamp: float = 0.0
        self._last_pose_stamp: float = 0.0
        self._control_status_text: str = ""
        self._path_complete_flag: bool = False

        # ------------------------------------------------------------ #
        #  Publishers
        # ------------------------------------------------------------ #
        self._state_pub = self.create_publisher(String, "/mission/state", 10)
        self._goal_pub = self.create_publisher(
            PoseStamped, "/mission/goal", 10
        )
        self._feedback_pub = self.create_publisher(
            String, "/mission/feedback", 10
        )

        # ------------------------------------------------------------ #
        #  Subscribers (generic — tolerant of missing upstream nodes)
        # ------------------------------------------------------------ #
        # QoS profile matching MAVROS (BEST_EFFORT required)
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(
            String, "/control/status", self._control_status_cb, 10
        )
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._pose_cb,
            mavros_qos,
        )
        self.create_subscription(
            String, "/control/waypoint_reached", self._waypoint_reached_cb, 10
        )

        # ------------------------------------------------------------ #
        #  Services
        # ------------------------------------------------------------ #
        self.create_service(SetBool, "/mission/load", self._srv_load)
        self.create_service(Trigger, "/mission/start", self._srv_start)
        self.create_service(Trigger, "/mission/pause", self._srv_pause)
        self.create_service(Trigger, "/mission/resume", self._srv_resume)
        self.create_service(Trigger, "/mission/abort", self._srv_abort)
        self.create_service(Trigger, "/mission/rtl", self._srv_rtl)

        # ------------------------------------------------------------ #
        #  Timers
        # ------------------------------------------------------------ #
        fsm_hz = self.get_parameter("fsm_tick_rate").value
        goal_hz = self.get_parameter("goal_publish_rate").value
        health_hz = self.get_parameter("health_check_rate").value

        self.create_timer(1.0 / fsm_hz, self._fsm_tick)
        self.create_timer(1.0 / goal_hz, self._publish_goal)
        self.create_timer(1.0 / health_hz, self._health_check)
        self.create_timer(1.0, self._publish_state)

        # Pending load params (set by /mission/load, consumed by FSM)
        self._pending_strategy_name: Optional[str] = None
        self._pending_params: Dict[str, Any] = {}

        # ------------------------------------------------------------ #
        #  Auto-start (optional)
        # ------------------------------------------------------------ #
        if self.get_parameter("auto_start").value:
            delay = self.get_parameter("auto_start_delay").value
            self.get_logger().info(
                f"Auto-start enabled — will load & start in {delay:.0f}s"
            )
            self._auto_start_timer = self.create_timer(
                delay, self._auto_start_mission
            )

        self.get_logger().info(
            "Mission Manager started — state: IDLE.  "
            f"Available strategies: {list(STRATEGY_REGISTRY.keys())}"
        )

    # ================================================================= #
    #  Subscriber callbacks
    # ================================================================= #

    def _control_status_cb(self, msg: String) -> None:
        self._control_status_text = msg.data
        self._last_control_stamp = time.monotonic()

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._current_pose = msg
        self._last_pose_stamp = time.monotonic()
        # Log first pose received
        if not hasattr(self, '_pose_logged'):
            self.flog.info(f'First pose received: z={msg.pose.position.z:.2f}m')
            self._pose_logged = True

    def _waypoint_reached_cb(self, msg: String) -> None:
        """React to path_complete from control to advance strategy goal."""
        if msg.data == 'path_complete' and self._state == MissionState.EXECUTING:
            self._path_complete_flag = True

    # ================================================================= #
    #  Auto-start
    # ================================================================= #

    def _auto_start_mission(self) -> None:
        """One-shot: load the configured strategy and start the mission."""
        # Cancel so this only fires once
        self._auto_start_timer.cancel()

        if self._state != MissionState.IDLE:
            self.get_logger().warn(
                f"Auto-start skipped — already in {self._state.name}"
            )
            return

        strat_name = (
            self.get_parameter("strategy_name")
            .get_parameter_value()
            .string_value
        )
        params_json = (
            self.get_parameter("strategy_params")
            .get_parameter_value()
            .string_value
        )
        try:
            params = json.loads(params_json)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Auto-start: bad strategy_params JSON: {exc}")
            return

        # Build a dummy response object to reuse _load_strategy
        resp = SetBool.Response()
        self._load_strategy(strat_name, params, resp)
        if not resp.success:
            self.get_logger().error(f"Auto-start load failed: {resp.message}")
            return

        self.get_logger().info(f"Auto-start: strategy '{strat_name}' loaded")
        self._transition(MissionState.PREFLIGHT)
        self.get_logger().info("Auto-start: mission started (PREFLIGHT)")

    # ================================================================= # #
    #  FSM core
    # ================================================================= #

    def _transition(self, target: MissionState) -> bool:
        """Attempt to transition to *target*.  Returns success."""
        if self._state == target:
            return True
        if not is_valid_transition(self._state, target):
            self.get_logger().warn(
                f"Invalid transition {self._state.name} → {target.name}.  "
                f"Valid: {[s.name for s in get_valid_transitions(self._state)]}"
            )
            return False
        old = self._state
        self._state = target
        self.get_logger().info(f"State: {old.name} → {target.name}")
        self.flog.info(f'FSM: {old.name} -> {target.name}')
        self._publish_state()
        return True

    def _fsm_tick(self) -> None:
        """Main FSM evaluation — runs at fsm_tick_rate Hz."""
        if self._state == MissionState.IDLE:
            pass  # Nothing to do — waiting for /mission/start

        elif self._state == MissionState.PREFLIGHT:
            self._do_preflight()

        elif self._state == MissionState.TAKEOFF:
            self._do_takeoff()

        elif self._state == MissionState.EXECUTING:
            self._do_executing()

        elif self._state == MissionState.PAUSED:
            pass  # Hold position — goal publisher keeps sending last goal

        elif self._state == MissionState.RTL:
            self._do_rtl()

        elif self._state == MissionState.LANDING:
            self._do_landing()

        elif self._state == MissionState.EMERGENCY:
            self._do_emergency()

    # ------------------------------------------------------------ #
    #  State handlers
    # ------------------------------------------------------------ #

    def _do_preflight(self) -> None:
        """Run preflight checks and auto-advance to TAKEOFF.

        Generic: does NOT require any downstream node to be alive.
        If the control node is reporting, we check for MAVLink
        connection.  Otherwise we proceed optimistically.
        """
        ok = True
        reasons = []

        if self._strategy is None:
            ok = False
            reasons.append("no strategy loaded")

        # If control node is alive, check its status text
        if self._control_alive():
            status = self._control_status_text
            if "Waiting for MAVLink" in status:
                ok = False
                reasons.append("control: waiting for MAVLink")
        else:
            # Control node not heard from — warn but don't block
            reasons.append("control node not yet detected (proceeding)")

        if ok:
            self.get_logger().info("Preflight OK — transitioning to TAKEOFF")
            self._transition(MissionState.TAKEOFF)
        else:
            self._publish_feedback(f"Preflight: {', '.join(reasons)}")

    def _do_takeoff(self) -> None:
        """Wait for the drone to reach takeoff altitude.

        The actual takeoff command is sent by the control node's
        auto-control state machine.  We just monitor altitude here.
        If we don't have pose data, we wait.
        """
        if self._current_pose is None:
            self.flog.info('Takeoff: pose is None')
            self._publish_feedback("Takeoff: waiting for pose data")
            return

        alt = self._current_pose.pose.position.z
        target = self._takeoff_alt
        # Throttled altitude logging (every 2s)
        now_ns = self.get_clock().now().nanoseconds
        if not hasattr(self, '_last_takeoff_log') or (now_ns - self._last_takeoff_log) > 2_000_000_000:
            self.flog.info(f'Takeoff check: alt={alt:.2f}m, need={target * 0.70:.2f}m (target={target:.2f}m)')
            self._last_takeoff_log = now_ns
        if alt >= target * 0.70:
            self.get_logger().info(
                f"Takeoff complete (alt={alt:.2f}m ≥ {target * 0.70:.2f}m)"
            )
            self.flog.info(f'Takeoff complete: alt={alt:.2f}m')
            self._transition(MissionState.EXECUTING)
        else:
            self._publish_feedback(
                f"Takeoff: alt={alt:.2f}/{target:.2f}m"
            )

    def _do_executing(self) -> None:
        """Delegate to the active strategy: advance waypoints, check done."""
        if self._strategy is None:
            self.get_logger().error("EXECUTING with no strategy — aborting")
            self._transition(MissionState.EMERGENCY)
            return

        # Advance strategy on every tick so time-based phases (dwell)
        # can progress, plus on path_complete for position-based phases.
        if self._current_pose is not None:
            changed = self._strategy.advance(self._current_pose, self._wp_threshold)
            if changed:
                progress = self._strategy.get_progress()
                self.flog.info(f'Phase advanced: {progress}')
        self._path_complete_flag = False

        # Update current goal
        self._current_goal = self._strategy.get_next_goal(self._current_pose)
        if self._current_goal is not None:
            g = self._current_goal.pose.position
            # Throttled logging (every 5s)
            now_ns = self.get_clock().now().nanoseconds
            if not hasattr(self, '_last_goal_log') or (now_ns - self._last_goal_log) > 5_000_000_000:
                self.flog.info(f'Goal: ({g.x:.1f}, {g.y:.1f}, {g.z:.1f})')
                self._last_goal_log = now_ns

        # Check completion
        if self._strategy.is_complete():
            self.get_logger().info(
                f"Mission complete — {self._strategy.get_progress()}"
            )
            if self._strategy.should_rtl_on_complete():
                self._publish_feedback("Mission complete — returning to launch")
                self._transition(MissionState.RTL)
            else:
                # Land in-place (e.g. package delivery: land at dropoff)
                landing_pos = self._strategy.get_landing_position()
                if landing_pos is not None:
                    self._landing_position = landing_pos
                elif self._current_pose is not None:
                    # Fall back to current position
                    self._landing_position = make_pose(
                        self._current_pose.pose.position.x,
                        self._current_pose.pose.position.y,
                        0.0,
                        self._frame_id,
                    )
                else:
                    self._landing_position = None
                self._publish_feedback("Mission complete — landing at destination")
                self._transition(MissionState.LANDING)
            return

        self._publish_feedback(self._strategy.get_progress())

    def _do_rtl(self) -> None:
        """Fly back to launch position, then transition to LANDING."""
        rtl_goal = make_pose(
            self._rtl_x, self._rtl_y, self._takeoff_alt, self._frame_id
        )
        self._current_goal = rtl_goal

        if self._current_pose is not None:
            dist = distance_xy(self._current_pose, rtl_goal)
            if dist < self._wp_threshold:
                self.get_logger().info("RTL position reached — landing")
                self._transition(MissionState.LANDING)
            else:
                self._publish_feedback(f"RTL: {dist:.1f}m to home")
        else:
            self._publish_feedback("RTL: waiting for pose data")

    def _do_landing(self) -> None:
        """Descend to ground — publish a ground-level goal.

        Landing is ultimately handled by the control node / ArduPilot.
        We set the goal altitude to 0 and wait for alt < 0.3 m, then
        go back to IDLE.

        If ``_landing_position`` is set (by a non-RTL strategy), land
        there instead of the RTL home position.
        """
        if self._landing_position is not None:
            land_goal = make_pose(
                self._landing_position.pose.position.x,
                self._landing_position.pose.position.y,
                0.0,
                self._frame_id,
            )
        else:
            land_goal = make_pose(
                self._rtl_x, self._rtl_y, 0.0, self._frame_id
            )
        self._current_goal = land_goal

        if self._current_pose is not None:
            alt = self._current_pose.pose.position.z
            if alt < 0.3:
                self.get_logger().info("Landed — returning to IDLE")
                self._cleanup_mission()
                self._transition(MissionState.IDLE)
            else:
                self._publish_feedback(f"Landing: alt={alt:.2f}m")
        else:
            self._publish_feedback("Landing: waiting for pose data")

    def _do_emergency(self) -> None:
        """Emergency state — hold position and wait for operator.

        The operator must call /mission/abort again or the system
        can be restarted.  Once the drone is on the ground, transition
        back to IDLE.
        """
        if self._current_pose is not None:
            # Hold current position
            self._current_goal = self._current_pose
            alt = self._current_pose.pose.position.z
            if alt < 0.3:
                self.get_logger().info("Emergency: on ground → IDLE")
                self._cleanup_mission()
                self._transition(MissionState.IDLE)

    # ================================================================= #
    #  Service handlers
    # ================================================================= #

    def _srv_load(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Load a mission strategy.

        The ``data`` field is repurposed: ``True`` = load, ``False`` = unload.

        Strategy name and parameters are passed via ROS parameters on this
        node.  Set them before calling the service:

            ros2 param set /mission_manager_node strategy_name "waypoint_survey"
            ros2 param set /mission_manager_node strategy_params \
                '{"min_x": 0, "min_y": 0, "max_x": 10, "max_y": 10}'

        Alternatively, for simple testing, the service auto-loads the
        strategy named in the ``strategy_name`` parameter with params
        from ``strategy_params`` (JSON string).
        """
        if not request.data:
            self._cleanup_mission()
            response.success = True
            response.message = "Mission unloaded"
            return response

        if self._state != MissionState.IDLE:
            response.success = False
            response.message = (
                f"Cannot load while in {self._state.name} — "
                "abort or land first"
            )
            return response

        # Read strategy name/params from ROS parameters
        strat_name = (
            self.get_parameter("strategy_name").get_parameter_value().string_value
        )
        params_json = (
            self.get_parameter("strategy_params")
            .get_parameter_value()
            .string_value
        )
        try:
            params = json.loads(params_json)
        except json.JSONDecodeError as exc:
            response.success = False
            response.message = f"Invalid JSON in strategy_params: {exc}"
            return response

        return self._load_strategy(strat_name, params, response)

    def _load_strategy(
        self,
        name: str,
        params: Dict[str, Any],
        response: SetBool.Response,
    ) -> SetBool.Response:
        cls = STRATEGY_REGISTRY.get(name)
        if cls is None:
            response.success = False
            response.message = (
                f"Unknown strategy '{name}'.  "
                f"Available: {list(STRATEGY_REGISTRY.keys())}"
            )
            return response

        strategy = cls()
        # Inject takeoff altitude if not specified
        params.setdefault("altitude", self._takeoff_alt)
        params.setdefault("frame_id", self._frame_id)

        if not strategy.initialize(params):
            response.success = False
            response.message = f"Strategy '{name}' rejected params: {params}"
            return response

        self._strategy = strategy

        # If it's a delivery strategy, inject the command publisher
        if hasattr(strategy, 'set_delivery_publisher'):
            if not hasattr(self, '_delivery_cmd_pub'):
                self._delivery_cmd_pub = self.create_publisher(
                    String, "/delivery/command", 10
                )
            strategy.set_delivery_publisher(self._delivery_cmd_pub)

        self.get_logger().info(f"Loaded strategy: {name}")
        response.success = True
        response.message = f"Strategy '{name}' loaded — call /mission/start"
        return response

    def _srv_start(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self._state != MissionState.IDLE:
            response.success = False
            response.message = f"Cannot start from {self._state.name}"
            return response
        if self._strategy is None:
            response.success = False
            response.message = "No strategy loaded — call /mission/load first"
            return response

        self._transition(MissionState.PREFLIGHT)
        response.success = True
        response.message = "Mission starting — preflight checks running"
        return response

    def _srv_pause(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self._state != MissionState.EXECUTING:
            response.success = False
            response.message = f"Cannot pause from {self._state.name}"
            return response

        if self._strategy:
            self._strategy.on_pause()
        self._transition(MissionState.PAUSED)
        response.success = True
        response.message = "Mission paused"
        return response

    def _srv_resume(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self._state != MissionState.PAUSED:
            response.success = False
            response.message = f"Cannot resume from {self._state.name}"
            return response

        if self._strategy:
            self._strategy.on_resume()
        self._transition(MissionState.EXECUTING)
        response.success = True
        response.message = "Mission resumed"
        return response

    def _srv_abort(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self._transition(MissionState.EMERGENCY)
        response.success = True
        response.message = "EMERGENCY — mission aborted"
        return response

    def _srv_rtl(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self._state not in (MissionState.EXECUTING, MissionState.PAUSED):
            response.success = False
            response.message = f"Cannot RTL from {self._state.name}"
            return response

        self._transition(MissionState.RTL)
        response.success = True
        response.message = "Returning to launch"
        return response

    # ================================================================= #
    #  Publishing helpers
    # ================================================================= #

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self._state.name
        self._state_pub.publish(msg)

    def _publish_feedback(self, text: str) -> None:
        msg = String()
        msg.data = f"[{self._state.name}] {text}"
        self._feedback_pub.publish(msg)

    def _publish_goal(self) -> None:
        """Publish the current goal to /mission/goal at a steady rate."""
        if self._current_goal is None:
            return
        # Only publish goals when in states that should move the drone
        if self._state in (
            MissionState.EXECUTING,
            MissionState.RTL,
            MissionState.LANDING,
            MissionState.EMERGENCY,
        ):
            self._current_goal.header.stamp = self.get_clock().now().to_msg()
            self._goal_pub.publish(self._current_goal)

    # ================================================================= #
    #  Health monitoring
    # ================================================================= #

    def _control_alive(self) -> bool:
        if self._last_control_stamp == 0.0:
            return False
        return (time.monotonic() - self._last_control_stamp) < self._health_timeout

    def _pose_alive(self) -> bool:
        if self._last_pose_stamp == 0.0:
            return False
        return (time.monotonic() - self._last_pose_stamp) < self._health_timeout

    def _health_check(self) -> None:
        """Periodic health monitoring — log warnings, do NOT auto-abort."""
        if self._state in (MissionState.IDLE,):
            return

        warnings = []
        if not self._control_alive():
            warnings.append("control node: no heartbeat")
        if not self._pose_alive():
            warnings.append("pose data: stale or missing")

        if warnings:
            self.get_logger().warn(f"Health: {', '.join(warnings)}")

    # ================================================================= #
    #  Helpers
    # ================================================================= #

    def _cleanup_mission(self) -> None:
        """Reset strategy and goal after mission ends."""
        if self._strategy:
            self._strategy.reset()
        self._strategy = None
        self._current_goal = None
        self._landing_position = None
        self._path_complete_flag = False


# =================================================================== #
#  Entry point
# =================================================================== #

def main(args=None):
    """ROS 2 entry point."""
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
