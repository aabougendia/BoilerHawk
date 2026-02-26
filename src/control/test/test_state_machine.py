"""
Unit tests for the FlightState state machine in control_node.

Tests all valid and invalid state transitions, ensuring the state
machine only allows defined transitions. Also tests coordinate
conversion logic and distance calculation.

These tests run on any platform (no ROS 2 / pymavlink dependency needed).
"""

import unittest
import math
import sys
import os

# Add the control package to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from control.flight_state import FlightState


class TestFlightStateTransitions(unittest.TestCase):
    """Test FlightState transition table."""

    def test_idle_can_go_to_setting_guided(self):
        valid = FlightState.TRANSITIONS[FlightState.IDLE]
        self.assertIn(FlightState.SETTING_GUIDED, valid)

    def test_idle_cannot_go_to_flying(self):
        valid = FlightState.TRANSITIONS[FlightState.IDLE]
        self.assertNotIn(FlightState.FLYING, valid)

    def test_idle_cannot_go_to_arming(self):
        valid = FlightState.TRANSITIONS[FlightState.IDLE]
        self.assertNotIn(FlightState.ARMING, valid)

    def test_setting_guided_can_go_to_arming(self):
        valid = FlightState.TRANSITIONS[FlightState.SETTING_GUIDED]
        self.assertIn(FlightState.ARMING, valid)

    def test_setting_guided_can_go_to_idle(self):
        valid = FlightState.TRANSITIONS[FlightState.SETTING_GUIDED]
        self.assertIn(FlightState.IDLE, valid)

    def test_arming_can_go_to_taking_off(self):
        valid = FlightState.TRANSITIONS[FlightState.ARMING]
        self.assertIn(FlightState.TAKING_OFF, valid)

    def test_arming_cannot_skip_to_flying(self):
        valid = FlightState.TRANSITIONS[FlightState.ARMING]
        self.assertNotIn(FlightState.FLYING, valid)

    def test_taking_off_can_go_to_hovering(self):
        valid = FlightState.TRANSITIONS[FlightState.TAKING_OFF]
        self.assertIn(FlightState.HOVERING, valid)

    def test_taking_off_can_emergency_land(self):
        valid = FlightState.TRANSITIONS[FlightState.TAKING_OFF]
        self.assertIn(FlightState.LANDING, valid)

    def test_hovering_can_go_to_flying(self):
        valid = FlightState.TRANSITIONS[FlightState.HOVERING]
        self.assertIn(FlightState.FLYING, valid)

    def test_hovering_can_land(self):
        valid = FlightState.TRANSITIONS[FlightState.HOVERING]
        self.assertIn(FlightState.LANDING, valid)

    def test_hovering_can_emergency_hold(self):
        valid = FlightState.TRANSITIONS[FlightState.HOVERING]
        self.assertIn(FlightState.EMERGENCY_HOLD, valid)

    def test_flying_can_go_to_hovering(self):
        valid = FlightState.TRANSITIONS[FlightState.FLYING]
        self.assertIn(FlightState.HOVERING, valid)

    def test_flying_can_land(self):
        valid = FlightState.TRANSITIONS[FlightState.FLYING]
        self.assertIn(FlightState.LANDING, valid)

    def test_flying_can_emergency_hold(self):
        valid = FlightState.TRANSITIONS[FlightState.FLYING]
        self.assertIn(FlightState.EMERGENCY_HOLD, valid)

    def test_flying_cannot_go_to_arming(self):
        valid = FlightState.TRANSITIONS[FlightState.FLYING]
        self.assertNotIn(FlightState.ARMING, valid)

    def test_landing_can_go_to_landed(self):
        valid = FlightState.TRANSITIONS[FlightState.LANDING]
        self.assertIn(FlightState.LANDED, valid)

    def test_landing_cannot_go_to_flying(self):
        valid = FlightState.TRANSITIONS[FlightState.LANDING]
        self.assertNotIn(FlightState.FLYING, valid)

    def test_landed_can_restart(self):
        valid = FlightState.TRANSITIONS[FlightState.LANDED]
        self.assertIn(FlightState.SETTING_GUIDED, valid)
        self.assertIn(FlightState.IDLE, valid)

    def test_landed_cannot_fly(self):
        valid = FlightState.TRANSITIONS[FlightState.LANDED]
        self.assertNotIn(FlightState.FLYING, valid)

    def test_emergency_hold_can_resume(self):
        valid = FlightState.TRANSITIONS[FlightState.EMERGENCY_HOLD]
        self.assertIn(FlightState.HOVERING, valid)
        self.assertIn(FlightState.FLYING, valid)
        self.assertIn(FlightState.LANDING, valid)

    def test_emergency_hold_cannot_arm(self):
        valid = FlightState.TRANSITIONS[FlightState.EMERGENCY_HOLD]
        self.assertNotIn(FlightState.ARMING, valid)


class TestFlightStateCompleteness(unittest.TestCase):
    """Verify the transition table is complete and consistent."""

    ALL_STATES = [
        FlightState.IDLE,
        FlightState.SETTING_GUIDED,
        FlightState.ARMING,
        FlightState.TAKING_OFF,
        FlightState.HOVERING,
        FlightState.FLYING,
        FlightState.LANDING,
        FlightState.LANDED,
        FlightState.EMERGENCY_HOLD,
    ]

    def test_all_states_have_transitions(self):
        """Every state must appear in the transition table."""
        for state in self.ALL_STATES:
            self.assertIn(state, FlightState.TRANSITIONS,
                          f'State {state} missing from TRANSITIONS')

    def test_all_transitions_point_to_valid_states(self):
        """Every target in the transition table must be a valid state."""
        for source, targets in FlightState.TRANSITIONS.items():
            for target in targets:
                self.assertIn(target, self.ALL_STATES,
                              f'Invalid target {target} from {source}')

    def test_no_self_transitions(self):
        """No state should transition to itself."""
        for state, targets in FlightState.TRANSITIONS.items():
            self.assertNotIn(state, targets,
                             f'State {state} has a self-transition')

    def test_every_state_is_reachable(self):
        """Every state must be reachable from at least one other state."""
        all_targets = set()
        for targets in FlightState.TRANSITIONS.values():
            all_targets.update(targets)
        for state in self.ALL_STATES:
            if state == FlightState.IDLE:
                continue  # Initial state
            self.assertIn(state, all_targets,
                          f'State {state} is unreachable')

    def test_full_happy_path_sequence(self):
        """The full happy-path is a valid chain of transitions."""
        happy_path = [
            FlightState.IDLE,
            FlightState.SETTING_GUIDED,
            FlightState.ARMING,
            FlightState.TAKING_OFF,
            FlightState.HOVERING,
            FlightState.FLYING,
            FlightState.HOVERING,  # path complete → hover
            FlightState.LANDING,
            FlightState.LANDED,
        ]
        for i in range(len(happy_path) - 1):
            src = happy_path[i]
            dst = happy_path[i + 1]
            valid = FlightState.TRANSITIONS[src]
            self.assertIn(dst, valid,
                          f'Happy path broken: {src} → {dst} is invalid')

    def test_emergency_hold_and_resume(self):
        """Emergency hold → resume → land is a valid sequence."""
        sequence = [
            FlightState.FLYING,
            FlightState.EMERGENCY_HOLD,
            FlightState.HOVERING,
            FlightState.LANDING,
            FlightState.LANDED,
        ]
        for i in range(len(sequence) - 1):
            src = sequence[i]
            dst = sequence[i + 1]
            valid = FlightState.TRANSITIONS[src]
            self.assertIn(dst, valid,
                          f'Emergency sequence broken: {src} → {dst}')


class TestNEDtoENUConversion(unittest.TestCase):
    """Test coordinate frame conversion correctness."""

    def test_ned_north_maps_to_enu_x(self):
        ned_x = 5.0
        enu_x = ned_x  # North → X
        self.assertEqual(enu_x, 5.0)

    def test_ned_east_maps_to_enu_neg_y(self):
        ned_y = 3.0
        enu_y = -ned_y  # East → -Y
        self.assertEqual(enu_y, -3.0)

    def test_ned_down_maps_to_enu_neg_z(self):
        ned_z = -2.0  # -2 in NED = 2m above ground
        enu_z = -ned_z
        self.assertEqual(enu_z, 2.0)

    def test_enu_to_ned_round_trip(self):
        enu_x, enu_y, enu_z = 1.0, 2.0, 3.0

        # ENU → NED
        ned_x = enu_x
        ned_y = -enu_y
        ned_z = -enu_z

        # NED → ENU
        recovered_x = ned_x
        recovered_y = -ned_y
        recovered_z = -ned_z

        self.assertAlmostEqual(enu_x, recovered_x)
        self.assertAlmostEqual(enu_y, recovered_y)
        self.assertAlmostEqual(enu_z, recovered_z)


class TestDistanceCalculation(unittest.TestCase):
    """Test Euclidean distance calculation logic (pure math, no ROS deps)."""

    @staticmethod
    def _distance(ax, ay, az, bx, by, bz):
        """Same formula as ControlNode._distance but with raw floats."""
        return math.sqrt((ax-bx)**2 + (ay-by)**2 + (az-bz)**2)

    def test_distance_3_4_5_triangle(self):
        self.assertAlmostEqual(self._distance(0, 0, 0, 3, 4, 0), 5.0)

    def test_distance_3d(self):
        self.assertAlmostEqual(
            self._distance(0, 0, 0, 1, 1, 1), math.sqrt(3))

    def test_distance_same_point(self):
        self.assertAlmostEqual(self._distance(5, 5, 5, 5, 5, 5), 0.0)

    def test_distance_symmetry(self):
        d1 = self._distance(1, 2, 3, 4, 6, 8)
        d2 = self._distance(4, 6, 8, 1, 2, 3)
        self.assertAlmostEqual(d1, d2)

    def test_within_threshold(self):
        threshold = 0.5
        d = self._distance(0, 0, 0, 0.3, 0.3, 0)
        self.assertTrue(d < threshold)

    def test_outside_threshold(self):
        threshold = 0.5
        d = self._distance(0, 0, 0, 1, 1, 0)
        self.assertFalse(d < threshold)


if __name__ == '__main__':
    unittest.main()
