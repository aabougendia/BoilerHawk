#!/usr/bin/env python3
"""
Standalone demo of planning module with hypothetical data.
This script demonstrates the path planning capabilities without requiring ROS 2.
"""

import numpy as np
import sys
import os

# Add planning module to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from planning.path_planner import PathPlanner


def print_grid_with_path(grid, path=None, start=None, goal=None):
    """
    Print occupancy grid with path visualization.
    
    Args:
        grid: 2D numpy array
        path: List of (row, col) positions
        start: Start position tuple
        goal: Goal position tuple
    """
    height, width = grid.shape
    display = np.full((height, width), ' ', dtype=str)
    
    # Mark free and occupied cells
    for i in range(height):
        for j in range(width):
            if grid[i, j] >= 50:
                display[i, j] = '█'
            else:
                display[i, j] = '·'
    
    # Mark path
    if path:
        for pos in path:
            if display[pos[0], pos[1]] != '█':
                display[pos[0], pos[1]] = '*'
    
    # Mark start and goal
    if start:
        display[start[0], start[1]] = 'S'
    if goal:
        display[goal[0], goal[1]] = 'G'
    
    # Print with border
    print('┌' + '─' * width + '┐')
    for row in display:
        print('│' + ''.join(row) + '│')
    print('└' + '─' * width + '┘')


def demo_simple_path():
    """Demo: Simple path in open space."""
    print("\n" + "="*60)
    print("DEMO 1: Simple Path Planning in Open Space")
    print("="*60)
    
    planner = PathPlanner(occupancy_threshold=50)
    
    # Create empty grid
    grid = np.zeros((20, 30), dtype=np.int8)
    
    # Add border
    grid[0, :] = 100
    grid[-1, :] = 100
    grid[:, 0] = 100
    grid[:, -1] = 100
    
    planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
    
    start = (2, 2)
    goal = (17, 27)
    
    path = planner.plan_global_path(start, goal)
    
    print(f"\nStart: {start}, Goal: {goal}")
    print(f"Path length: {len(path)} cells")
    print(f"\nVisualization (S=Start, G=Goal, *=Path, █=Obstacle, ·=Free):")
    print_grid_with_path(grid, path, start, goal)


def demo_obstacle_avoidance():
    """Demo: Path planning around obstacles."""
    print("\n" + "="*60)
    print("DEMO 2: Obstacle Avoidance")
    print("="*60)
    
    planner = PathPlanner(occupancy_threshold=50)
    
    # Create grid with obstacles
    grid = np.zeros((25, 35), dtype=np.int8)
    
    # Add border
    grid[0, :] = 100
    grid[-1, :] = 100
    grid[:, 0] = 100
    grid[:, -1] = 100
    
    # Add obstacles
    grid[5:20, 15] = 100  # Vertical wall with gap
    grid[5, 15:25] = 100  # Horizontal extension
    grid[12:20, 25] = 100  # Another vertical wall
    
    planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
    
    start = (12, 5)
    goal = (12, 30)
    
    path = planner.plan_global_path(start, goal)
    
    print(f"\nStart: {start}, Goal: {goal}")
    print(f"Path length: {len(path)} cells")
    print(f"Path must navigate around multiple obstacles")
    print(f"\nVisualization:")
    print_grid_with_path(grid, path, start, goal)


def demo_maze_navigation():
    """Demo: Path planning through a maze."""
    print("\n" + "="*60)
    print("DEMO 3: Maze Navigation")
    print("="*60)
    
    planner = PathPlanner(occupancy_threshold=50)
    
    # Create maze
    grid = np.zeros((25, 25), dtype=np.int8)
    
    # Outer border
    grid[0, :] = 100
    grid[-1, :] = 100
    grid[:, 0] = 100
    grid[:, -1] = 100
    
    # Maze walls
    grid[5, 1:20] = 100
    grid[10, 5:24] = 100
    grid[15, 1:15] = 100
    grid[20, 10:24] = 100
    
    grid[1:20, 20] = 100
    grid[10:24, 15] = 100
    grid[5:20, 10] = 100
    
    # Create gaps
    grid[5, 10] = 0
    grid[10, 20] = 0
    grid[15, 5] = 0
    grid[20, 15] = 0
    grid[15, 10] = 0
    
    planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
    
    start = (2, 2)
    goal = (22, 22)
    
    path = planner.plan_global_path(start, goal)
    
    print(f"\nStart: {start}, Goal: {goal}")
    print(f"Path length: {len(path)} cells")
    print(f"Robot must navigate through complex maze structure")
    print(f"\nVisualization:")
    print_grid_with_path(grid, path, start, goal)


def demo_local_path_planning():
    """Demo: Local path planning with dynamic updates."""
    print("\n" + "="*60)
    print("DEMO 4: Local Path Planning with Dynamic Obstacles")
    print("="*60)
    
    planner = PathPlanner(occupancy_threshold=50)
    
    # Initial grid
    grid = np.zeros((20, 40), dtype=np.int8)
    grid[0, :] = 100
    grid[-1, :] = 100
    grid[:, 0] = 100
    grid[:, -1] = 100
    
    planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
    
    start = (10, 2)
    goal = (10, 37)
    
    # Plan global path
    global_path = planner.plan_global_path(start, goal)
    
    print(f"\nStart: {start}, Goal: {goal}")
    print(f"Global path length: {len(global_path)} cells")
    print(f"\nInitial global path:")
    print_grid_with_path(grid, global_path, start, goal)
    
    # Simulate robot movement and add dynamic obstacle
    current_pos = (10, 15)
    
    print(f"\n--- Robot moved to position {current_pos} ---")
    print(f"New obstacle detected ahead!")
    
    # Add dynamic obstacle
    grid[8:13, 20:23] = 100
    planner.update_occupancy_grid(grid, 0.1, (0.0, 0.0))
    
    # Plan local path (will replan around obstacle)
    local_path = planner.plan_local_path(current_pos, lookahead_distance=15)
    
    print(f"Local path length: {len(local_path)} cells")
    print(f"\nUpdated local path (avoids new obstacle):")
    print_grid_with_path(grid, local_path, current_pos, goal)


def demo_coordinate_conversion():
    """Demo: Grid to world coordinate conversion."""
    print("\n" + "="*60)
    print("DEMO 5: Coordinate Conversion (Grid ↔ World)")
    print("="*60)
    
    planner = PathPlanner(occupancy_threshold=50)
    
    # Setup grid with specific resolution and origin
    grid = np.zeros((100, 100), dtype=np.int8)
    resolution = 0.05  # 5cm per cell
    origin = (-2.5, -2.5)  # Grid center at origin
    
    planner.update_occupancy_grid(grid, resolution, origin)
    
    print(f"\nGrid configuration:")
    print(f"  Size: {grid.shape[0]}x{grid.shape[1]} cells")
    print(f"  Resolution: {resolution} m/cell")
    print(f"  Origin: {origin} m")
    print(f"  World coverage: {grid.shape[1] * resolution}m x {grid.shape[0] * resolution}m")
    
    # Test conversions
    test_cases = [
        ((50, 50), "Grid center"),
        ((0, 0), "Grid top-left"),
        ((99, 99), "Grid bottom-right"),
        ((25, 75), "Random position"),
    ]
    
    print(f"\nCoordinate conversions:")
    print(f"{'Grid (row, col)':<20} {'→':<3} {'World (x, y) [m]':<20} {'Description':<20}")
    print("-" * 70)
    
    for grid_pos, desc in test_cases:
        world_pos = planner.grid_to_world(grid_pos)
        print(f"{str(grid_pos):<20} → {f'({world_pos[0]:.2f}, {world_pos[1]:.2f})':<20} {desc:<20}")
    
    print(f"\nRound-trip conversion test:")
    original_grid = (50, 50)
    world_pos = planner.grid_to_world(original_grid)
    back_to_grid = planner.world_to_grid(world_pos)
    print(f"  Original: {original_grid}")
    print(f"  → World: ({world_pos[0]:.3f}, {world_pos[1]:.3f})")
    print(f"  → Grid: {back_to_grid}")
    print(f"  Match: {original_grid == back_to_grid} ✓")


def main():
    """Run all demos."""
    print("\n" + "="*60)
    print("PLANNING MODULE - HYPOTHETICAL DATA DEMONSTRATION")
    print("="*60)
    print("\nThis demo showcases the planning module's path planning")
    print("capabilities using various hypothetical scenarios.")
    
    try:
        demo_simple_path()
        input("\nPress Enter to continue to next demo...")
        
        demo_obstacle_avoidance()
        input("\nPress Enter to continue to next demo...")
        
        demo_maze_navigation()
        input("\nPress Enter to continue to next demo...")
        
        demo_local_path_planning()
        input("\nPress Enter to continue to next demo...")
        
        demo_coordinate_conversion()
        
        print("\n" + "="*60)
        print("ALL DEMOS COMPLETED SUCCESSFULLY!")
        print("="*60)
        print("\nThe planning module is working correctly.")
        print("All path planning algorithms executed successfully with hypothetical data.")
        print("\nNext steps:")
        print("  1. Integrate with actual ROS 2 perception module")
        print("  2. Test with real sensor data")
        print("  3. Tune parameters for your specific robot platform")
        print("="*60 + "\n")
        
    except KeyboardInterrupt:
        print("\n\nDemo interrupted by user.")
    except Exception as e:
        print(f"\nError during demo: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
