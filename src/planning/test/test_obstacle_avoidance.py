"""
Tests for obstacle avoidance and replanning logic.

Verifies:
- is_path_blocked() detects occupied cells on a path
- Danger detection proximity checks (safety_radius)
- Replanning produces valid paths around new obstacles
- Emergency hold triggering and clearing

These tests use only PathPlanner (no ROS 2 dependency).
"""

import unittest
import math
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from planning.path_planner import PathPlanner


class TestIsPathBlocked(unittest.TestCase):
    """Test PathPlanner.is_path_blocked() method."""

    def setUp(self):
        self.planner = PathPlanner(occupancy_threshold=50)
        self.grid = np.zeros((20, 20), dtype=np.int8)
        self.planner.update_occupancy_grid(self.grid, 0.1, (0.0, 0.0))

    def test_empty_path_not_blocked(self):
        self.assertFalse(self.planner.is_path_blocked([]))

    def test_clear_path_not_blocked(self):
        path = [(5, 5), (5, 6), (5, 7), (5, 8)]
        self.assertFalse(self.planner.is_path_blocked(path))

    def test_path_through_obstacle_is_blocked(self):
        self.grid[5, 7] = 100
        self.planner.update_occupancy_grid(self.grid, 0.1, (0.0, 0.0))
        path = [(5, 5), (5, 6), (5, 7), (5, 8)]
        self.assertTrue(self.planner.is_path_blocked(path))

    def test_path_near_but_not_through_obstacle(self):
        self.grid[4, 7] = 100  # Obstacle one row above the path
        self.planner.update_occupancy_grid(self.grid, 0.1, (0.0, 0.0))
        path = [(5, 5), (5, 6), (5, 7), (5, 8)]
        self.assertFalse(self.planner.is_path_blocked(path))

    def test_path_out_of_bounds_is_blocked(self):
        path = [(5, 5), (5, 6), (-1, 0)]  # Out of bounds cell
        self.assertTrue(self.planner.is_path_blocked(path))

    def test_path_with_single_blocked_cell(self):
        self.grid[10, 10] = 100
        self.planner.update_occupancy_grid(self.grid, 0.1, (0.0, 0.0))
        path = [(10, 10)]
        self.assertTrue(self.planner.is_path_blocked(path))

    def test_path_with_threshold_boundary(self):
        """Cell exactly at threshold should be blocked."""
        self.grid[5, 5] = 50  # Exactly at threshold
        self.planner.update_occupancy_grid(self.grid, 0.1, (0.0, 0.0))
        path = [(5, 5)]
        self.assertTrue(self.planner.is_path_blocked(path))

    def test_path_below_threshold_not_blocked(self):
        """Cell just below threshold should not be blocked."""
        self.grid[5, 5] = 49  # Below threshold
        self.planner.update_occupancy_grid(self.grid, 0.1, (0.0, 0.0))
        path = [(5, 5)]
        self.assertFalse(self.planner.is_path_blocked(path))


class TestDangerDetectionLogic(unittest.TestCase):
    """Test the proximity-based danger detection algorithm.

    We test the pure math here — the actual _check_danger method
    needs ROS 2, but its core algorithm is: find minimum distance
    from drone to any occupied cell, compare to safety_radius.
    """

    def setUp(self):
        self.planner = PathPlanner(occupancy_threshold=50)
        self.safety_radius = 1.0  # meters

    def _min_obstacle_distance(self, drone_pos_grid, grid_data):
        """Replicate the danger detection distance calculation."""
        self.planner.update_occupancy_grid(grid_data, 0.1, (0.0, 0.0))
        drone_world = self.planner.grid_to_world(drone_pos_grid)

        occupied = np.argwhere(grid_data >= self.planner.occupancy_threshold)
        if len(occupied) == 0:
            return float('inf')

        min_dist = float('inf')
        for cell in occupied:
            row, col = int(cell[0]), int(cell[1])
            cell_world = self.planner.grid_to_world((row, col))
            dx = drone_world[0] - cell_world[0]
            dy = drone_world[1] - cell_world[1]
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < min_dist:
                min_dist = dist
        return min_dist

    def test_no_obstacles_returns_inf(self):
        grid = np.zeros((20, 20), dtype=np.int8)
        dist = self._min_obstacle_distance((10, 10), grid)
        self.assertEqual(dist, float('inf'))

    def test_far_obstacle_above_safety_radius(self):
        grid = np.zeros((100, 100), dtype=np.int8)
        # Obstacle 20 cells away at 0.1m resolution = 2.0m
        grid[10, 30] = 100
        dist = self._min_obstacle_distance((10, 10), grid)
        self.assertGreater(dist, self.safety_radius)

    def test_close_obstacle_below_safety_radius(self):
        grid = np.zeros((100, 100), dtype=np.int8)
        # Obstacle 3 cells away at 0.1m resolution = 0.3m
        grid[10, 13] = 100
        dist = self._min_obstacle_distance((10, 10), grid)
        self.assertLess(dist, self.safety_radius)

    def test_obstacle_at_exact_safety_radius(self):
        grid = np.zeros((100, 100), dtype=np.int8)
        # 10 cells at 0.1m = 1.0m (exactly at safety_radius)
        grid[10, 20] = 100
        dist = self._min_obstacle_distance((10, 10), grid)
        # Distance should be close to 1.0m
        self.assertAlmostEqual(dist, 1.0, delta=0.2)

    def test_multiple_obstacles_returns_closest(self):
        grid = np.zeros((100, 100), dtype=np.int8)
        grid[10, 50] = 100  # Far obstacle
        grid[10, 13] = 100  # Close obstacle (3 cells = 0.3m)
        dist = self._min_obstacle_distance((10, 10), grid)
        self.assertLess(dist, 0.5)  # Should pick the close one

    def test_obstacle_directly_on_drone(self):
        grid = np.zeros((20, 20), dtype=np.int8)
        grid[10, 10] = 100
        dist = self._min_obstacle_distance((10, 10), grid)
        self.assertAlmostEqual(dist, 0.0, delta=0.01)


class TestDynamicReplanScenarios(unittest.TestCase):
    """Test complex replanning scenarios with dynamic obstacles."""

    def setUp(self):
        self.planner = PathPlanner(occupancy_threshold=50)

    def test_obstacle_appears_then_disappears(self):
        """Path should re-clear when obstacle is removed."""
        grid = np.zeros((25, 25), dtype=np.int8)
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))

        # Initial clear path
        path1 = self.planner.plan_global_path((2, 2), (22, 22))
        self.assertGreater(len(path1), 0)
        self.assertFalse(self.planner.is_path_blocked(path1))

        # Add obstacle
        grid[12, 12] = 100
        grid[13, 13] = 100
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
        self.assertTrue(self.planner.is_path_blocked(path1))

        # Remove obstacle
        grid[12, 12] = 0
        grid[13, 13] = 0
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
        self.assertFalse(self.planner.is_path_blocked(path1))

    def test_obstacle_forces_longer_path(self):
        """Adding obstacle should make the replanned path longer."""
        grid = np.zeros((25, 25), dtype=np.int8)
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))

        # Direct path
        path1 = self.planner.plan_global_path((2, 2), (22, 22))
        len1 = len(path1)

        # Add large obstacle on the diagonal
        grid[10:15, 10:15] = 100
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))

        # Replanned path should be longer (has to go around)
        path2 = self.planner.plan_global_path((2, 2), (22, 22))
        self.assertGreater(len(path2), 0)
        self.assertGreater(len(path2), len1)

    def test_multiple_obstacles_still_finds_path(self):
        """Path planner should navigate around multiple obstacles."""
        grid = np.zeros((30, 30), dtype=np.int8)

        # Multiple scattered obstacles
        grid[5:8, 5:8] = 100
        grid[15:18, 15:18] = 100
        grid[10, 20:25] = 100
        grid[20:25, 10] = 100

        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
        path = self.planner.plan_global_path((0, 0), (29, 29))
        self.assertGreater(len(path), 0)

        # Path should avoid all obstacles
        for pos in path:
            self.assertTrue(self.planner.is_valid_cell(pos))

    def test_growing_obstacle(self):
        """Obstacle that grows over time should trigger successive replans."""
        grid = np.zeros((30, 30), dtype=np.int8)
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))

        paths = []
        for size in range(1, 5):
            grid_copy = np.zeros((30, 30), dtype=np.int8)
            # Growing obstacle
            grid_copy[13-size:13+size, 13-size:13+size] = 100
            self.planner.update_occupancy_grid(grid_copy, 0.1, (0.0, 0.0))

            path = self.planner.plan_global_path((2, 2), (27, 27))
            if len(path) > 0:
                # Path should always be valid
                for pos in path:
                    self.assertTrue(self.planner.is_valid_cell(pos))
                paths.append(path)

        # Paths should generally get longer as obstacle grows
        if len(paths) >= 2:
            self.assertGreaterEqual(len(paths[-1]), len(paths[0]))


class TestEmergencyHoldLogic(unittest.TestCase):
    """Test the emergency hold state machine interaction."""

    def test_emergency_hold_is_valid_from_flying(self):
        """Import FlightState and verify FLYING → EMERGENCY_HOLD."""
        sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                        '..', '..', 'control'))
        from control.flight_state import FlightState
        valid = FlightState.TRANSITIONS[FlightState.FLYING]
        self.assertIn(FlightState.EMERGENCY_HOLD, valid)

    def test_emergency_hold_can_resume_to_hovering(self):
        sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                        '..', '..', 'control'))
        from control.flight_state import FlightState
        valid = FlightState.TRANSITIONS[FlightState.EMERGENCY_HOLD]
        self.assertIn(FlightState.HOVERING, valid)

    def test_emergency_hold_can_land(self):
        sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                        '..', '..', 'control'))
        from control.flight_state import FlightState
        valid = FlightState.TRANSITIONS[FlightState.EMERGENCY_HOLD]
        self.assertIn(FlightState.LANDING, valid)


if __name__ == '__main__':
    unittest.main()
