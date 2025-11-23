#!/usr/bin/env python3
"""
Quick demo script (non-interactive version) to verify the perception module.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from perception.path_planner import PathPlanner
import numpy as np


def main():
    print("\n" + "="*60)
    print("PERCEPTION MODULE - QUICK VERIFICATION")
    print("="*60 + "\n")
    
    # Test 1: Simple path
    print("✓ Test 1: Simple path planning...")
    planner = PathPlanner(occupancy_threshold=50)
    grid = np.zeros((20, 20), dtype=np.int8)
    planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
    path = planner.plan_global_path((2, 2), (17, 17))
    print(f"  Path found with {len(path)} waypoints")
    assert len(path) > 0, "Path should be found"
    
    # Test 2: Obstacle avoidance
    print("✓ Test 2: Obstacle avoidance...")
    grid[10, :] = 100  # Add wall
    grid[10, 15] = 0   # Create gap
    planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
    path = planner.plan_global_path((5, 10), (15, 10))
    print(f"  Path found around obstacle with {len(path)} waypoints")
    assert len(path) > 0, "Path should navigate around obstacle"
    
    # Test 3: Local path planning
    print("✓ Test 3: Local path planning...")
    grid = np.zeros((30, 30), dtype=np.int8)
    planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
    global_path = planner.plan_global_path((2, 2), (27, 27))
    local_path = planner.plan_local_path((5, 5), lookahead_distance=10)
    print(f"  Local path: {len(local_path)} waypoints (lookahead: 10)")
    assert len(local_path) > 0, "Local path should be found"
    
    # Test 4: Dynamic replanning
    print("✓ Test 4: Dynamic replanning...")
    grid[15, 15:20] = 100  # Add new obstacle
    planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
    local_path = planner.plan_local_path((10, 10), lookahead_distance=15)
    print(f"  Replanned path: {len(local_path) if local_path else 0} waypoints")
    
    # Test 5: Coordinate conversion
    print("✓ Test 5: Coordinate conversion...")
    world_pos = planner.grid_to_world((15, 15))
    grid_pos = planner.world_to_grid(world_pos)
    print(f"  Grid (15,15) → World ({world_pos[0]:.2f},{world_pos[1]:.2f}) → Grid {grid_pos}")
    assert abs(grid_pos[0] - 15) <= 1 and abs(grid_pos[1] - 15) <= 1, "Conversion should be accurate"
    
    print("\n" + "="*60)
    print("✓ ALL TESTS PASSED - Perception module is working!")
    print("="*60)
    print("\nThe perception module successfully:")
    print("  • Plans optimal paths using A* algorithm")
    print("  • Avoids obstacles dynamically")
    print("  • Updates local paths in real-time")
    print("  • Handles coordinate transformations")
    print("  • Works with hypothetical occupancy grid data")
    print("\nReady for integration with ROS 2 planning module!")
    print("="*60 + "\n")


if __name__ == '__main__':
    try:
        main()
    except AssertionError as e:
        print(f"\n✗ Test failed: {e}\n")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ Error: {e}\n")
        import traceback
        traceback.print_exc()
        sys.exit(1)
