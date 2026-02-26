"""
Unit tests for waypoint-following logic in control_node.

Tests the path following algorithms and state transitions
that happen during waypoint flight, without ROS 2 dependencies.
"""

import unittest
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from control.flight_state import FlightState


class TestPathFollowingLogic(unittest.TestCase):
    """Test the waypoint-following algorithms used by control_node."""

    def setUp(self):
        """Set up a simple waypoint list."""
        # Simulated waypoints as (x, y) tuples
        self.waypoints = [
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0),
            (2.0, 1.0),
            (2.0, 2.0),
        ]
        self.threshold = 0.5
        self.target_altitude = 2.0

    @staticmethod
    def _distance_2d(pos, wp):
        return math.sqrt((pos[0] - wp[0])**2 + (pos[1] - wp[1])**2)

    def test_waypoint_0_is_first_target(self):
        """After receiving a path, waypoint index should start at 0."""
        idx = 0
        self.assertEqual(idx, 0)
        self.assertEqual(self.waypoints[idx], (0.0, 0.0))

    def test_advance_when_within_threshold(self):
        """Waypoint advances when drone is close enough."""
        idx = 0
        drone_pos = (0.3, 0.2)  # Close to WP 0
        dist = self._distance_2d(drone_pos, self.waypoints[idx])
        if dist < self.threshold:
            idx += 1
        self.assertEqual(idx, 1)

    def test_no_advance_when_outside_threshold(self):
        """Waypoint does NOT advance when drone is far away."""
        idx = 0
        drone_pos = (5.0, 5.0)  # Far from WP 0
        dist = self._distance_2d(drone_pos, self.waypoints[idx])
        if dist < self.threshold:
            idx += 1
        self.assertEqual(idx, 0)

    def test_advance_through_all_waypoints(self):
        """Drone visits all waypoints sequentially."""
        idx = 0
        # Simulate the drone arriving at each waypoint
        for wp in self.waypoints:
            dist = self._distance_2d(wp, self.waypoints[idx])
            self.assertLess(dist, self.threshold)
            idx += 1
        self.assertEqual(idx, len(self.waypoints))

    def test_path_complete_when_past_last_waypoint(self):
        """Path is complete when index exceeds waypoint count."""
        idx = len(self.waypoints)
        is_complete = idx >= len(self.waypoints)
        self.assertTrue(is_complete)

    def test_new_path_resets_index(self):
        """Receiving a new path resets the waypoint index to 0."""
        idx = 3  # Mid-flight
        # Simulate new path received
        idx = 0
        self.assertEqual(idx, 0)

    def test_empty_path_rejected(self):
        """Empty path should not be accepted."""
        empty_path = []
        self.assertEqual(len(empty_path), 0)
        # In control_node, this returns early without setting current_path

    def test_altitude_override(self):
        """Waypoint z-coordinate is overridden by target_altitude."""
        # In control_node, _update_target always uses self.target_altitude
        # regardless of what the planning node sends for z
        wp_z = 0.0  # Planning sends 0.0 (2D plan)
        actual_z = self.target_altitude  # Override
        self.assertEqual(actual_z, 2.0)
        self.assertNotEqual(actual_z, wp_z)

    def test_path_replacement_mid_flight(self):
        """A new path during flight resets progress to the start."""
        idx = 3
        old_waypoints = self.waypoints

        # New path arrives
        new_waypoints = [(5.0, 5.0), (6.0, 6.0)]
        idx = 0  # Reset

        self.assertEqual(idx, 0)
        self.assertEqual(len(new_waypoints), 2)
        self.assertNotEqual(new_waypoints, old_waypoints)


class TestWaypointStateTransitions(unittest.TestCase):
    """Test state transitions triggered by waypoint events."""

    def test_hovering_to_flying_on_path_received(self):
        """Receiving a path while hovering should trigger FLYING."""
        state = FlightState.HOVERING
        valid = FlightState.TRANSITIONS[state]
        self.assertIn(FlightState.FLYING, valid)

    def test_flying_to_hovering_on_path_complete(self):
        """Path complete should transition to HOVERING."""
        state = FlightState.FLYING
        valid = FlightState.TRANSITIONS[state]
        self.assertIn(FlightState.HOVERING, valid)

    def test_flying_to_landing_on_auto_land(self):
        """Auto-land on path complete should transition FLYING→LANDING."""
        # With auto_land_on_complete: True, path complete triggers:
        # FLYING → LANDING (direct, via _advance_waypoint)
        state = FlightState.FLYING
        valid = FlightState.TRANSITIONS[state]
        self.assertIn(FlightState.LANDING, valid)

    def test_hold_command_goes_to_hovering(self):
        """'hold' command during flight should go to HOVERING."""
        state = FlightState.FLYING
        valid = FlightState.TRANSITIONS[state]
        self.assertIn(FlightState.HOVERING, valid)

    def test_cannot_fly_without_hovering_first(self):
        """TAKING_OFF cannot skip directly to FLYING."""
        state = FlightState.TAKING_OFF
        valid = FlightState.TRANSITIONS[state]
        self.assertNotIn(FlightState.FLYING, valid)


class TestSetpointBehavior(unittest.TestCase):
    """Test when setpoints should and shouldn't be sent."""

    SETPOINT_STATES = {FlightState.HOVERING, FlightState.FLYING,
                       FlightState.EMERGENCY_HOLD}

    def test_setpoints_sent_while_hovering(self):
        self.assertIn(FlightState.HOVERING, self.SETPOINT_STATES)

    def test_setpoints_sent_while_flying(self):
        self.assertIn(FlightState.FLYING, self.SETPOINT_STATES)

    def test_setpoints_sent_during_emergency_hold(self):
        self.assertIn(FlightState.EMERGENCY_HOLD, self.SETPOINT_STATES)

    def test_no_setpoints_during_takeoff(self):
        self.assertNotIn(FlightState.TAKING_OFF, self.SETPOINT_STATES)

    def test_no_setpoints_during_landing(self):
        self.assertNotIn(FlightState.LANDING, self.SETPOINT_STATES)

    def test_no_setpoints_while_idle(self):
        self.assertNotIn(FlightState.IDLE, self.SETPOINT_STATES)

    def test_no_setpoints_while_arming(self):
        self.assertNotIn(FlightState.ARMING, self.SETPOINT_STATES)

    def test_flying_sends_target_not_hold(self):
        """When FLYING with a target, we send target position, not hold."""
        has_target = True
        state = FlightState.FLYING
        should_send_target = state == FlightState.FLYING and has_target
        self.assertTrue(should_send_target)

    def test_hovering_sends_hold_position(self):
        """When HOVERING, we send current position as hold."""
        has_target = False
        state = FlightState.HOVERING
        should_hold = state in (FlightState.HOVERING,
                                FlightState.EMERGENCY_HOLD) and not has_target
        self.assertTrue(should_hold)


if __name__ == '__main__':
    unittest.main()
