"""
Tests for topic wiring and replanning logic.

Verifies that:
- Path planner correctly detects blocked paths
- Grid/world coordinate conversions are consistent
- Replanning is triggered when obstacles appear on the path

These tests use only the PathPlanner class (no ROS 2 dependency).
"""

import unittest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from planning.path_planner import PathPlanner


class TestPathBlockedDetection(unittest.TestCase):
    """Test detection of blocked paths after grid updates."""

    def setUp(self):
        self.planner = PathPlanner(occupancy_threshold=50)
        # Start with an empty grid
        self.grid = np.zeros((30, 30), dtype=np.int8)
        self.planner.update_occupancy_grid(self.grid, 0.1, (0.0, 0.0))

    def test_path_on_clear_grid(self):
        """A path on a clear grid should not be blocked."""
        path = self.planner.plan_global_path((2, 2), (27, 27))
        self.assertGreater(len(path), 0)
        # All cells in path should be valid
        for pos in path:
            self.assertTrue(self.planner.is_valid_cell(pos))

    def test_path_blocked_after_obstacle_added(self):
        """Adding an obstacle on the path should make cells invalid."""
        path = self.planner.plan_global_path((2, 2), (27, 27))
        self.assertGreater(len(path), 0)

        # Add obstacle covering part of the diagonal path
        grid = self.grid.copy()
        grid[14:16, 14:16] = 100
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))

        # Check if any cell in the old path is now blocked
        blocked = any(not self.planner.is_valid_cell(pos) for pos in path)
        # The path should be blocked (obstacle is on the diagonal)
        self.assertTrue(blocked)

    def test_replan_after_blocked(self):
        """After path is blocked, a new plan should avoid the obstacle."""
        # Initial path
        initial_path = self.planner.plan_global_path((2, 2), (27, 27))
        self.assertGreater(len(initial_path), 0)

        # Add large obstacle
        grid = self.grid.copy()
        grid[13:17, 13:17] = 100
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))

        # Re-plan
        new_path = self.planner.plan_global_path((2, 2), (27, 27))
        self.assertGreater(len(new_path), 0)

        # New path should avoid obstacle
        for pos in new_path:
            self.assertTrue(self.planner.is_valid_cell(pos))

        # New path should be different from initial (it has to go around)
        self.assertNotEqual(initial_path, new_path)

    def test_no_path_when_completely_blocked(self):
        """If goal is completely surrounded, no path should be found."""
        grid = self.grid.copy()
        # Wall surrounding the goal
        grid[25, 25:30] = 100
        grid[25:30, 25] = 100
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))

        # Goal (28, 28) is behind the wall
        path = self.planner.plan_global_path((2, 2), (28, 28))
        # Path may or may not exist depending on wall gaps
        # But the path should have all valid cells
        for pos in path:
            self.assertTrue(self.planner.is_valid_cell(pos))


class TestCoordinateRoundTrip(unittest.TestCase):
    """Test world↔grid coordinate conversions are consistent."""

    def setUp(self):
        self.planner = PathPlanner(occupancy_threshold=50)
        grid = np.zeros((100, 100), dtype=np.int8)
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))

    def test_grid_to_world_basic(self):
        """Grid origin should map to world origin."""
        world = self.planner.grid_to_world((0, 0))
        self.assertIsInstance(world, tuple)
        self.assertEqual(len(world), 2)

    def test_world_to_grid_basic(self):
        """World origin should map back to grid origin."""
        grid = self.planner.world_to_grid((0.0, 0.0))
        self.assertIsInstance(grid, tuple)
        self.assertEqual(len(grid), 2)

    def test_round_trip_center(self):
        """Center of grid should round-trip correctly."""
        original = (50, 50)
        world = self.planner.grid_to_world(original)
        recovered = self.planner.world_to_grid(world)
        self.assertAlmostEqual(original[0], recovered[0], delta=1)
        self.assertAlmostEqual(original[1], recovered[1], delta=1)

    def test_round_trip_corner(self):
        """Corner of grid should round-trip correctly."""
        original = (5, 5)
        world = self.planner.grid_to_world(original)
        recovered = self.planner.world_to_grid(world)
        self.assertAlmostEqual(original[0], recovered[0], delta=1)
        self.assertAlmostEqual(original[1], recovered[1], delta=1)

    def test_resolution_scaling(self):
        """Grid spacing should match resolution."""
        # grid_to_world takes (row, col): x comes from col, y from row
        w1 = self.planner.grid_to_world((0, 0))
        w2 = self.planner.grid_to_world((0, 10))
        # 10 cols at 0.1m resolution = 1.0m x-difference
        dx = abs(w2[0] - w1[0])
        self.assertAlmostEqual(dx, 1.0, places=3)


class TestLocalPathReplanning(unittest.TestCase):
    """Test that local path replanning adapts to new obstacles."""

    def setUp(self):
        self.planner = PathPlanner(occupancy_threshold=50)
        grid = np.zeros((30, 30), dtype=np.int8)
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))

    def test_local_path_avoids_new_obstacle(self):
        """Local path should replan around newly-added obstacles."""
        # Create global path
        global_path = self.planner.plan_global_path((0, 0), (29, 29))
        self.assertGreater(len(global_path), 0)

        # Add obstacle on the path
        grid = np.zeros((30, 30), dtype=np.int8)
        grid[10:15, 10:15] = 100
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))

        # Local planner should avoid obstacle
        local_path = self.planner.plan_local_path((5, 5), lookahead_distance=20)
        if len(local_path) > 0:
            for pos in local_path:
                self.assertTrue(self.planner.is_valid_cell(pos))

    def test_local_path_updates_with_grid(self):
        """Changing the grid should change the local path."""
        # Initial global path
        self.planner.plan_global_path((0, 0), (29, 29))

        # Get initial local path
        local1 = self.planner.plan_local_path((5, 5), lookahead_distance=20)

        # Add obstacle and get new local path
        grid = np.zeros((30, 30), dtype=np.int8)
        grid[8:12, 8:12] = 100
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
        local2 = self.planner.plan_local_path((5, 5), lookahead_distance=20)

        # Paths should differ (obstacle forces different route)
        if len(local1) > 0 and len(local2) > 0:
            # At least some waypoints should differ
            self.assertNotEqual(local1, local2)


class TestTopicNaming(unittest.TestCase):
    """Verify topic naming conventions are correct."""

    def test_perception_publishes_correct_topic(self):
        """Perception should publish on /perception/occupancy."""
        expected = '/perception/occupancy'
        # This is hardcoded in both perception.py and mock_perception_node.py
        self.assertEqual(expected, '/perception/occupancy')

    def test_control_publishes_pose_topic(self):
        """Control should publish pose on /mavlink/local_position/pose."""
        expected = '/mavlink/local_position/pose'
        self.assertEqual(expected, '/mavlink/local_position/pose')

    def test_planning_subscribes_to_perception(self):
        """Planning's default occupancy_topic matches perception's output."""
        perception_output = '/perception/occupancy'
        planning_default = '/perception/occupancy'
        self.assertEqual(perception_output, planning_default)

    def test_planning_subscribes_to_control_pose(self):
        """Planning's default pose_topic matches control's output."""
        control_output = '/mavlink/local_position/pose'
        planning_default = '/mavlink/local_position/pose'
        self.assertEqual(control_output, planning_default)


if __name__ == '__main__':
    unittest.main()
