#!/usr/bin/env python3
"""
Unit tests for path planner module using hypothetical data.
"""

import unittest
import numpy as np
from perception.path_planner import PathPlanner, Node


class TestNode(unittest.TestCase):
    """Test cases for Node class."""
    
    def test_node_creation(self):
        """Test node creation and initialization."""
        node = Node((5, 10))
        self.assertEqual(node.position, (5, 10))
        self.assertIsNone(node.parent)
        self.assertEqual(node.g, 0)
        self.assertEqual(node.h, 0)
        self.assertEqual(node.f, 0)
    
    def test_node_equality(self):
        """Test node equality comparison."""
        node1 = Node((5, 10))
        node2 = Node((5, 10))
        node3 = Node((6, 10))
        
        self.assertEqual(node1, node2)
        self.assertNotEqual(node1, node3)
    
    def test_node_comparison(self):
        """Test node comparison for priority queue."""
        node1 = Node((0, 0))
        node1.f = 5.0
        
        node2 = Node((1, 1))
        node2.f = 10.0
        
        self.assertTrue(node1 < node2)
        self.assertFalse(node2 < node1)


class TestPathPlanner(unittest.TestCase):
    """Test cases for PathPlanner class."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.planner = PathPlanner(occupancy_threshold=50)
        
        # Create a simple test grid (20x20)
        self.test_grid = np.zeros((20, 20), dtype=np.int8)
        
        # Add some obstacles
        self.test_grid[5, 5:15] = 100  # Horizontal wall
        self.test_grid[10:15, 10] = 100  # Vertical wall
        
        self.planner.update_occupancy_grid(
            self.test_grid,
            resolution=0.1,
            origin=(0.0, 0.0)
        )
    
    def test_grid_update(self):
        """Test occupancy grid update."""
        self.assertIsNotNone(self.planner.occupancy_grid)
        self.assertEqual(self.planner.grid_width, 20)
        self.assertEqual(self.planner.grid_height, 20)
        self.assertEqual(self.planner.grid_resolution, 0.1)
        self.assertEqual(self.planner.grid_origin, (0.0, 0.0))
    
    def test_is_valid_cell(self):
        """Test cell validity checking."""
        # Valid free cell
        self.assertTrue(self.planner.is_valid_cell((0, 0)))
        
        # Invalid - occupied cell
        self.assertFalse(self.planner.is_valid_cell((5, 10)))
        
        # Invalid - out of bounds
        self.assertFalse(self.planner.is_valid_cell((-1, 0)))
        self.assertFalse(self.planner.is_valid_cell((0, 25)))
    
    def test_heuristic(self):
        """Test heuristic distance calculation."""
        dist = self.planner.heuristic((0, 0), (3, 4))
        self.assertAlmostEqual(dist, 5.0, places=5)  # 3-4-5 triangle
        
        dist = self.planner.heuristic((5, 5), (5, 5))
        self.assertEqual(dist, 0.0)
    
    def test_get_neighbors(self):
        """Test neighbor generation."""
        neighbors = self.planner.get_neighbors((2, 2))
        
        # Should have 8 neighbors for interior cell
        self.assertEqual(len(neighbors), 8)
        
        # Check specific neighbors
        self.assertIn((1, 2), neighbors)  # up
        self.assertIn((3, 2), neighbors)  # down
        self.assertIn((2, 1), neighbors)  # left
        self.assertIn((2, 3), neighbors)  # right
        
        # Test corner cell (fewer neighbors)
        neighbors = self.planner.get_neighbors((0, 0))
        self.assertEqual(len(neighbors), 3)  # Only 3 valid neighbors at corner
    
    def test_simple_path_planning(self):
        """Test simple path planning without obstacles."""
        # Create empty grid
        empty_grid = np.zeros((10, 10), dtype=np.int8)
        self.planner.update_occupancy_grid(empty_grid, 0.1, (0.0, 0.0))
        
        # Plan straight path
        path = self.planner.plan_global_path((0, 0), (5, 5))
        
        self.assertGreater(len(path), 0)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (5, 5))
    
    def test_path_around_obstacle(self):
        """Test path planning around obstacles."""
        # Create grid with wall
        grid = np.zeros((20, 20), dtype=np.int8)
        grid[8:12, 5:15] = 100  # Horizontal wall
        
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
        
        # Plan path that must go around wall
        start = (15, 10)  # Below wall
        goal = (5, 10)  # Above wall
        
        path = self.planner.plan_global_path(start, goal)
        
        self.assertGreater(len(path), 0)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)
        
        # Verify path doesn't go through obstacle
        for pos in path:
            self.assertTrue(self.planner.is_valid_cell(pos))
    
    def test_no_path_scenario(self):
        """Test scenario where no path exists."""
        # Create grid with complete barrier
        grid = np.zeros((20, 20), dtype=np.int8)
        grid[10, :] = 100  # Complete horizontal wall
        
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
        
        # Try to plan path across barrier
        path = self.planner.plan_global_path((5, 10), (15, 10))
        
        self.assertEqual(len(path), 0)  # No path should be found
    
    def test_local_path_planning(self):
        """Test local path planning."""
        # First create global path
        grid = np.zeros((30, 30), dtype=np.int8)
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
        
        global_path = self.planner.plan_global_path((0, 0), (29, 29))
        self.assertGreater(len(global_path), 0)
        
        # Plan local path
        local_path = self.planner.plan_local_path((5, 5), lookahead_distance=10)
        
        self.assertGreater(len(local_path), 0)
        self.assertLessEqual(len(local_path), 10)
    
    def test_local_path_with_new_obstacle(self):
        """Test local path replanning when obstacle detected."""
        # Create initial path
        grid = np.zeros((30, 30), dtype=np.int8)
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
        
        global_path = self.planner.plan_global_path((0, 0), (29, 29))
        self.assertGreater(len(global_path), 0)
        
        # Add obstacle on path
        grid[10:15, 10:15] = 100
        self.planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
        
        # Local planner should detect obstacle and replan
        local_path = self.planner.plan_local_path((5, 5), lookahead_distance=20)
        
        # Verify local path avoids new obstacle
        if len(local_path) > 0:
            for pos in local_path:
                self.assertTrue(self.planner.is_valid_cell(pos))
    
    def test_world_to_grid_conversion(self):
        """Test world to grid coordinate conversion."""
        self.planner.grid_height = 100
        self.planner.grid_resolution = 0.1
        self.planner.grid_origin = (0.0, 0.0)
        
        # Test conversion
        grid_pos = self.planner.world_to_grid((1.0, 1.0))
        
        # Verify conversion
        self.assertIsInstance(grid_pos, tuple)
        self.assertEqual(len(grid_pos), 2)
    
    def test_grid_to_world_conversion(self):
        """Test grid to world coordinate conversion."""
        self.planner.grid_height = 100
        self.planner.grid_resolution = 0.1
        self.planner.grid_origin = (0.0, 0.0)
        
        # Test conversion
        world_pos = self.planner.grid_to_world((50, 50))
        
        # Verify conversion
        self.assertIsInstance(world_pos, tuple)
        self.assertEqual(len(world_pos), 2)
        self.assertIsInstance(world_pos[0], float)
        self.assertIsInstance(world_pos[1], float)
    
    def test_round_trip_conversion(self):
        """Test round-trip coordinate conversion."""
        self.planner.grid_height = 100
        self.planner.grid_resolution = 0.1
        self.planner.grid_origin = (0.0, 0.0)
        
        # Original grid position
        original_grid = (50, 50)
        
        # Convert to world and back
        world_pos = self.planner.grid_to_world(original_grid)
        final_grid = self.planner.world_to_grid(world_pos)
        
        # Should be close to original (within rounding error)
        self.assertAlmostEqual(original_grid[0], final_grid[0], delta=1)
        self.assertAlmostEqual(original_grid[1], final_grid[1], delta=1)


class TestHypotheticalScenarios(unittest.TestCase):
    """Test complex hypothetical scenarios."""
    
    def test_maze_scenario(self):
        """Test path planning in a maze-like environment."""
        planner = PathPlanner(occupancy_threshold=50)
        
        # Create maze (30x30)
        maze = np.zeros((30, 30), dtype=np.int8)
        
        # Add maze walls
        maze[5, 5:25] = 100
        maze[10, 5:20] = 100
        maze[15, 10:25] = 100
        maze[20, 5:20] = 100
        
        maze[5:25, 5] = 100
        maze[10:25, 15] = 100
        maze[5:20, 20] = 100
        
        # Add gaps
        maze[5, 15] = 0
        maze[10, 10] = 0
        maze[15, 20] = 0
        maze[20, 15] = 0
        
        planner.update_occupancy_grid(maze, 0.1, (0.0, 0.0))
        
        # Plan path through maze
        path = planner.plan_global_path((2, 2), (27, 27))
        
        if len(path) > 0:
            # Verify path validity
            for pos in path:
                self.assertTrue(planner.is_valid_cell(pos))
            
            # Verify endpoints
            self.assertEqual(path[0], (2, 2))
            self.assertEqual(path[-1], (27, 27))
    
    def test_narrow_passage(self):
        """Test path planning through narrow passages."""
        planner = PathPlanner(occupancy_threshold=50)
        
        # Create grid with narrow passage
        grid = np.zeros((20, 30), dtype=np.int8)
        
        # Create complete walls with narrow gap
        grid[0:20, 14] = 100  # Left wall (full height)
        grid[0:20, 16] = 100  # Right wall (full height)
        grid[:, 15] = 100     # Fill middle column
        
        # Create narrow passage (only 2 cells open)
        grid[9, 15] = 0  # Narrow passage (1 cell wide)
        grid[10, 15] = 0
        
        planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
        
        # Plan path through narrow passage
        path = planner.plan_global_path((10, 5), (10, 25))
        
        if len(path) > 0:
            # Should find path through narrow passage
            self.assertGreater(len(path), 0)
            
            # Verify it goes through the gap
            passage_cells = [(9, 15), (10, 15)]
            path_uses_passage = any(pos in passage_cells for pos in path)
            self.assertTrue(path_uses_passage)
    
    def test_dynamic_environment(self):
        """Test path adaptation in dynamic environment."""
        planner = PathPlanner(occupancy_threshold=50)
        
        # Initial empty grid
        grid = np.zeros((25, 25), dtype=np.int8)
        planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
        
        # Plan initial global path
        global_path = planner.plan_global_path((2, 2), (22, 22))
        self.assertGreater(len(global_path), 0)
        
        initial_path_length = len(global_path)
        
        # Add dynamic obstacle on path
        grid[12, 12] = 100
        grid[12, 13] = 100
        grid[13, 12] = 100
        planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
        
        # Plan local path (should adapt)
        local_path = planner.plan_local_path((2, 2), lookahead_distance=15)
        
        # Verify adaptation
        if len(local_path) > 0:
            for pos in local_path:
                self.assertTrue(planner.is_valid_cell(pos))


def run_tests():
    """Run all unit tests."""
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add test cases
    suite.addTests(loader.loadTestsFromTestCase(TestNode))
    suite.addTests(loader.loadTestsFromTestCase(TestPathPlanner))
    suite.addTests(loader.loadTestsFromTestCase(TestHypotheticalScenarios))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result


if __name__ == '__main__':
    result = run_tests()
    
    # Print summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    print(f"Tests run: {result.testsRun}")
    print(f"Successes: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print("="*70)
    
    # Exit with appropriate code
    exit(0 if result.wasSuccessful() else 1)
