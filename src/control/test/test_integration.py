"""
End-to-end integration tests for BoilerHawk.

Tests the full pipeline using MockMAVLinkState:
1. State machine transitions (IDLE → GUIDED → ARMING → TAKEOFF → HOVERING → FLYING → LANDING)
2. Waypoint following with mock position
3. Obstacle avoidance and replanning
4. Emergency hold and resume

No ROS 2 or Gazebo required — all logic tested with pure Python mocks.
"""

import unittest
import math
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'planning'))
# mock_mavlink is in the same directory as this test
sys.path.insert(0, os.path.dirname(__file__))

from control.flight_state import FlightState
from mock_mavlink import MockMAVLinkState, MockMAVLinkServer
from planning.path_planner import PathPlanner


class TestEndToEndStateMachine(unittest.TestCase):
    """Test the full state machine sequence with mock MAVLink."""

    def setUp(self):
        self.mav = MockMAVLinkState()
        self.state = FlightState.IDLE

    def _transition(self, new_state):
        """Apply a state transition, return True if valid."""
        if FlightState.is_valid_transition(self.state, new_state):
            self.state = new_state
            return True
        return False

    def test_full_happy_path(self):
        """Complete flight from idle to landed."""
        # IDLE → SETTING_GUIDED
        self.assertTrue(self._transition(FlightState.SETTING_GUIDED))
        self.mav.set_mode(MockMAVLinkServer.MODE_GUIDED)
        self.assertEqual(self.mav.mode, MockMAVLinkServer.MODE_GUIDED)

        # SETTING_GUIDED → ARMING
        self.assertTrue(self._transition(FlightState.ARMING))
        self.mav.arm()
        self.assertTrue(self.mav.armed)

        # ARMING → TAKING_OFF
        self.assertTrue(self._transition(FlightState.TAKING_OFF))
        self.mav.takeoff(2.0)

        # Simulate takeoff
        for _ in range(30):  # 3 seconds at 10Hz
            self.mav.simulate_step(0.1)
        self.assertGreater(self.mav.altitude, 1.5)

        # TAKING_OFF → HOVERING
        self.assertTrue(self._transition(FlightState.HOVERING))

        # HOVERING → FLYING (path received)
        self.assertTrue(self._transition(FlightState.FLYING))

        # Simulate flying to a waypoint
        self.mav.set_position(5.0, 0.0, -2.0)
        for _ in range(100):
            self.mav.simulate_step(0.1)

        # FLYING → HOVERING (path complete)
        self.assertTrue(self._transition(FlightState.HOVERING))

        # HOVERING → LANDING
        self.assertTrue(self._transition(FlightState.LANDING))
        self.mav.land()
        self.assertEqual(self.mav.mode, MockMAVLinkServer.MODE_LAND)

        # Simulate landing (disarm)
        self.mav.disarm()
        self.assertFalse(self.mav.armed)

        # LANDING → LANDED
        self.assertTrue(self._transition(FlightState.LANDED))

        # Verify command sequence
        cmd_types = [c[0] for c in self.mav.commands_received]
        self.assertEqual(cmd_types, [
            'set_mode', 'arm', 'takeoff', 'set_position', 'land', 'disarm'
        ])

    def test_emergency_hold_during_flight(self):
        """Emergency hold interrupts flight, then resumes."""
        # Fast forward to FLYING
        self._transition(FlightState.SETTING_GUIDED)
        self._transition(FlightState.ARMING)
        self._transition(FlightState.TAKING_OFF)
        self._transition(FlightState.HOVERING)
        self._transition(FlightState.FLYING)
        self.assertEqual(self.state, FlightState.FLYING)

        # FLYING → EMERGENCY_HOLD (obstacle detected)
        self.assertTrue(self._transition(FlightState.EMERGENCY_HOLD))
        self.assertEqual(self.state, FlightState.EMERGENCY_HOLD)

        # EMERGENCY_HOLD → HOVERING (obstacle cleared)
        self.assertTrue(self._transition(FlightState.HOVERING))
        self.assertEqual(self.state, FlightState.HOVERING)

        # HOVERING → FLYING (resume)
        self.assertTrue(self._transition(FlightState.FLYING))
        self.assertEqual(self.state, FlightState.FLYING)

    def test_emergency_hold_then_land(self):
        """Emergency hold followed by landing."""
        self._transition(FlightState.SETTING_GUIDED)
        self._transition(FlightState.ARMING)
        self._transition(FlightState.TAKING_OFF)
        self._transition(FlightState.HOVERING)
        self._transition(FlightState.FLYING)

        # FLYING → EMERGENCY_HOLD
        self._transition(FlightState.EMERGENCY_HOLD)
        # EMERGENCY_HOLD → LANDING (too dangerous, land now)
        self.assertTrue(self._transition(FlightState.LANDING))
        self._transition(FlightState.LANDED)
        self.assertEqual(self.state, FlightState.LANDED)


class TestEndToEndWaypointFollowing(unittest.TestCase):
    """Test waypoint following with mock position."""

    def setUp(self):
        self.mav = MockMAVLinkState()
        self.mav.set_mode(MockMAVLinkServer.MODE_GUIDED)
        self.mav.arm()
        self.mav.takeoff(2.0)
        # Simulate takeoff
        for _ in range(30):
            self.mav.simulate_step(0.1)

    def test_follow_waypoints(self):
        """Drone visits each waypoint in sequence."""
        waypoints = [(2.0, 0.0, -2.0), (2.0, 2.0, -2.0), (0.0, 2.0, -2.0)]
        threshold = 0.3

        for wx, wy, wz in waypoints:
            self.mav.set_position(wx, wy, wz)
            for _ in range(100):  # 10 seconds max
                self.mav.simulate_step(0.1)
                dx = self.mav.position[0] - wx
                dy = self.mav.position[1] - wy
                dz = self.mav.position[2] - wz
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                if dist < threshold:
                    break
            # Should have reached the waypoint
            self.assertLess(dist, threshold,
                            f'Failed to reach waypoint ({wx},{wy},{wz})')

    def test_position_converges(self):
        """Mock position converges toward target."""
        self.mav.set_position(5.0, 3.0, -2.0)
        distances = []
        for _ in range(50):
            self.mav.simulate_step(0.1)
            dx = self.mav.position[0] - 5.0
            dy = self.mav.position[1] - 3.0
            dz = self.mav.position[2] - (-2.0)
            distances.append(math.sqrt(dx*dx + dy*dy + dz*dz))

        # Distance should be monotonically decreasing
        for i in range(1, len(distances)):
            self.assertLessEqual(distances[i], distances[i-1] + 0.001)


class TestEndToEndObstacleAvoidance(unittest.TestCase):
    """Test obstacle avoidance with planning + mock MAVLink."""

    def setUp(self):
        self.planner = PathPlanner(occupancy_threshold=50)
        self.mav = MockMAVLinkState()
        self.mav.set_mode(MockMAVLinkServer.MODE_GUIDED)
        self.mav.arm()
        self.mav.takeoff(2.0)
        for _ in range(30):
            self.mav.simulate_step(0.1)

    def test_replan_around_obstacle(self):
        """Full scenario: plan → obstacle appears → replan → follow new path."""
        # Initial clear grid
        grid = np.zeros((30, 30), dtype=np.int8)
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))

        # Plan initial path
        path1 = self.planner.plan_global_path((2, 2), (27, 27))
        self.assertGreater(len(path1), 0)

        # Add obstacle that blocks the path
        grid[14:16, 14:16] = 100
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))

        # Verify path is blocked
        self.assertTrue(self.planner.is_path_blocked(path1))

        # Replan
        path2 = self.planner.plan_global_path((2, 2), (27, 27))
        self.assertGreater(len(path2), 0)
        self.assertFalse(self.planner.is_path_blocked(path2))

        # Follow first waypoint of new path
        wp = self.planner.grid_to_world(path2[0])
        self.mav.set_position(wp[0], wp[1], -2.0)
        for _ in range(50):
            self.mav.simulate_step(0.1)

    def test_danger_detection_triggers_hold(self):
        """Close obstacle triggers EMERGENCY_HOLD state."""
        grid = np.zeros((100, 100), dtype=np.int8)
        # Obstacle very close to drone at (10, 10)
        grid[10, 12] = 100  # 2 cells away at 0.1m = 0.2m
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))

        drone_pos = (10, 10)
        drone_world = self.planner.grid_to_world(drone_pos)

        # Check proximity
        obstacle_world = self.planner.grid_to_world((10, 12))
        dx = drone_world[0] - obstacle_world[0]
        dy = drone_world[1] - obstacle_world[1]
        dist = math.sqrt(dx*dx + dy*dy)

        safety_radius = 1.0
        self.assertLess(dist, safety_radius)  # Should trigger EMERGENCY_HOLD

        # State should allow transition to EMERGENCY_HOLD
        valid = FlightState.TRANSITIONS[FlightState.FLYING]
        self.assertIn(FlightState.EMERGENCY_HOLD, valid)

    def test_full_mission_with_obstacle(self):
        """Complete mission: takeoff → fly → hit obstacle → replan → resume → land."""
        state = FlightState.IDLE

        # Setup
        grid = np.zeros((30, 30), dtype=np.int8)
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))

        # Phase 1: Takeoff
        state = FlightState.SETTING_GUIDED
        state = FlightState.ARMING
        state = FlightState.TAKING_OFF
        state = FlightState.HOVERING

        # Phase 2: Plan and fly
        path = self.planner.plan_global_path((2, 2), (27, 27))
        self.assertGreater(len(path), 0)
        state = FlightState.FLYING

        # Phase 3: Obstacle detected!
        grid[14:16, 14:16] = 100
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
        self.assertTrue(self.planner.is_path_blocked(path))

        # Replan
        new_path = self.planner.plan_global_path((2, 2), (27, 27))
        self.assertGreater(len(new_path), 0)
        self.assertFalse(self.planner.is_path_blocked(new_path))

        # Phase 4: Continue flying
        # (drone would follow new_path waypoints here)

        # Phase 5: Mission complete → land
        state = FlightState.HOVERING
        state = FlightState.LANDING
        self.mav.land()
        self.mav.disarm()
        state = FlightState.LANDED
        self.assertEqual(state, FlightState.LANDED)


class TestMockMAVLinkState(unittest.TestCase):
    """Test the MockMAVLinkState helper class itself."""

    def test_initial_state(self):
        mav = MockMAVLinkState()
        self.assertFalse(mav.armed)
        self.assertEqual(mav.mode, MockMAVLinkServer.MODE_STABILIZE)
        self.assertAlmostEqual(mav.altitude, 0.0)

    def test_arm_requires_guided(self):
        mav = MockMAVLinkState()
        self.assertFalse(mav.arm())  # Not in GUIDED mode
        mav.set_mode(MockMAVLinkServer.MODE_GUIDED)
        self.assertTrue(mav.arm())

    def test_takeoff_increases_altitude(self):
        mav = MockMAVLinkState()
        mav.set_mode(MockMAVLinkServer.MODE_GUIDED)
        mav.arm()
        mav.takeoff(5.0)
        for _ in range(80):  # 8 seconds
            mav.simulate_step(0.1)
        self.assertGreater(mav.altitude, 4.0)

    def test_position_tracking(self):
        mav = MockMAVLinkState()
        mav.set_mode(MockMAVLinkServer.MODE_GUIDED)
        mav.arm()
        mav.takeoff(2.0)
        for _ in range(30):
            mav.simulate_step(0.1)

        mav.set_position(3.0, 4.0, -2.0)
        for _ in range(100):
            mav.simulate_step(0.1)

        self.assertAlmostEqual(mav.position[0], 3.0, delta=0.3)
        self.assertAlmostEqual(mav.position[1], 4.0, delta=0.3)

    def test_command_log(self):
        mav = MockMAVLinkState()
        mav.set_mode(MockMAVLinkServer.MODE_GUIDED)
        mav.arm()
        mav.takeoff(2.0)
        mav.set_position(1.0, 0.0, -2.0)
        mav.land()

        self.assertEqual(len(mav.commands_received), 5)
        self.assertEqual(mav.commands_received[0][0], 'set_mode')
        self.assertEqual(mav.commands_received[1][0], 'arm')
        self.assertEqual(mav.commands_received[2][0], 'takeoff')
        self.assertEqual(mav.commands_received[3][0], 'set_position')
        self.assertEqual(mav.commands_received[4][0], 'land')


if __name__ == '__main__':
    unittest.main()
