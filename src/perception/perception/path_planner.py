"""
Path planning utilities for perception module.
Implements A* algorithm for global path planning and local path refinement.
"""

import numpy as np
from typing import List, Tuple, Optional
import heapq


class Node:
    """Node class for A* pathfinding algorithm."""
    
    def __init__(self, position: Tuple[int, int], parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cost from start to current node
        self.h = 0  # Heuristic cost from current node to goal
        self.f = 0  # Total cost (g + h)
    
    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.f < other.f
    
    def __hash__(self):
        return hash(self.position)


class PathPlanner:
    """
    Path planner using A* algorithm for occupancy grid navigation.
    """
    
    def __init__(self, occupancy_threshold: int = 50):
        """
        Initialize path planner.
        
        Args:
            occupancy_threshold: Threshold for considering a cell occupied (0-100)
        """
        self.occupancy_threshold = occupancy_threshold
        self.global_path = []
        self.local_path = []
        self.occupancy_grid = None
        self.grid_resolution = 0.05  # meters per cell
        self.grid_width = 0
        self.grid_height = 0
        self.grid_origin = (0, 0)
    
    def update_occupancy_grid(self, grid_data: np.ndarray, resolution: float, 
                             origin: Tuple[float, float]):
        """
        Update the occupancy grid.
        
        Args:
            grid_data: 2D numpy array with occupancy values (0-100, -1 for unknown)
            resolution: Grid resolution in meters per cell
            origin: (x, y) coordinates of the grid origin
        """
        self.occupancy_grid = grid_data
        self.grid_resolution = resolution
        self.grid_height, self.grid_width = grid_data.shape
        self.grid_origin = origin
    
    def is_valid_cell(self, position: Tuple[int, int]) -> bool:
        """
        Check if a cell position is valid and not occupied.
        
        Args:
            position: (row, col) cell coordinates
            
        Returns:
            True if cell is valid and free
        """
        row, col = position
        
        # Check bounds
        if row < 0 or row >= self.grid_height or col < 0 or col >= self.grid_width:
            return False
        
        # Check if occupied
        if self.occupancy_grid[row, col] >= self.occupancy_threshold:
            return False
        
        # Unknown cells (-1) are considered valid for now
        return True
    
    def heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """
        Calculate heuristic distance between two positions (Euclidean distance).
        
        Args:
            pos1: First position (row, col)
            pos2: Second position (row, col)
            
        Returns:
            Euclidean distance
        """
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def get_neighbors(self, position: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Get valid neighboring cells (8-connected).
        
        Args:
            position: Current position (row, col)
            
        Returns:
            List of valid neighbor positions
        """
        row, col = position
        neighbors = []
        
        # 8-connected grid (including diagonals)
        directions = [
            (-1, 0),  # up
            (1, 0),   # down
            (0, -1),  # left
            (0, 1),   # right
            (-1, -1), # up-left
            (-1, 1),  # up-right
            (1, -1),  # down-left
            (1, 1)    # down-right
        ]
        
        for dr, dc in directions:
            neighbor_pos = (row + dr, col + dc)
            if self.is_valid_cell(neighbor_pos):
                neighbors.append(neighbor_pos)
        
        return neighbors
    
    def reconstruct_path(self, node: Node) -> List[Tuple[int, int]]:
        """
        Reconstruct path from goal node to start.
        
        Args:
            node: Goal node
            
        Returns:
            List of positions from start to goal
        """
        path = []
        current = node
        while current is not None:
            path.append(current.position)
            current = current.parent
        return path[::-1]  # Reverse to get start to goal
    
    def plan_global_path(self, start: Tuple[int, int], 
                        goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Plan global path using A* algorithm.
        
        Args:
            start: Start position (row, col)
            goal: Goal position (row, col)
            
        Returns:
            List of positions from start to goal, empty if no path found
        """
        if self.occupancy_grid is None:
            print("Error: Occupancy grid not set")
            return []
        
        if not self.is_valid_cell(start):
            print(f"Error: Start position {start} is not valid")
            return []
        
        if not self.is_valid_cell(goal):
            print(f"Error: Goal position {goal} is not valid")
            return []
        
        # Initialize start and goal nodes
        start_node = Node(start)
        goal_node = Node(goal)
        
        # Initialize open and closed lists
        open_list = []
        heapq.heappush(open_list, start_node)
        closed_set = set()
        open_dict = {start: start_node}
        
        while open_list:
            # Get node with lowest f score
            current_node = heapq.heappop(open_list)
            del open_dict[current_node.position]
            closed_set.add(current_node.position)
            
            # Check if reached goal
            if current_node == goal_node:
                self.global_path = self.reconstruct_path(current_node)
                return self.global_path
            
            # Check neighbors
            for neighbor_pos in self.get_neighbors(current_node.position):
                if neighbor_pos in closed_set:
                    continue
                
                # Calculate costs
                # Diagonal moves cost sqrt(2), straight moves cost 1
                move_cost = 1.414 if abs(neighbor_pos[0] - current_node.position[0]) + \
                                     abs(neighbor_pos[1] - current_node.position[1]) == 2 else 1.0
                
                tentative_g = current_node.g + move_cost
                
                # Check if neighbor already in open list
                if neighbor_pos in open_dict:
                    neighbor_node = open_dict[neighbor_pos]
                    if tentative_g < neighbor_node.g:
                        # Update node with better path
                        neighbor_node.g = tentative_g
                        neighbor_node.h = self.heuristic(neighbor_pos, goal)
                        neighbor_node.f = neighbor_node.g + neighbor_node.h
                        neighbor_node.parent = current_node
                        heapq.heapify(open_list)
                else:
                    # Create new node
                    neighbor_node = Node(neighbor_pos, current_node)
                    neighbor_node.g = tentative_g
                    neighbor_node.h = self.heuristic(neighbor_pos, goal)
                    neighbor_node.f = neighbor_node.g + neighbor_node.h
                    heapq.heappush(open_list, neighbor_node)
                    open_dict[neighbor_pos] = neighbor_node
        
        # No path found
        print("Warning: No path found from start to goal")
        self.global_path = []
        return []
    
    def plan_local_path(self, current_position: Tuple[int, int], 
                       lookahead_distance: int = 20) -> List[Tuple[int, int]]:
        """
        Plan local path based on global path and current position.
        Refines path based on recent occupancy grid updates.
        
        Args:
            current_position: Current robot position (row, col)
            lookahead_distance: Number of cells to look ahead in global path
            
        Returns:
            Local path segment
        """
        if not self.global_path:
            print("Warning: No global path available for local planning")
            return []
        
        # Find closest point on global path to current position
        min_dist = float('inf')
        closest_idx = 0
        
        for idx, pos in enumerate(self.global_path):
            dist = self.heuristic(current_position, pos)
            if dist < min_dist:
                min_dist = dist
                closest_idx = idx
        
        # Extract local segment from global path
        end_idx = min(closest_idx + lookahead_distance, len(self.global_path))
        local_segment = self.global_path[closest_idx:end_idx]
        
        # Check if local segment is still valid (no new obstacles)
        for pos in local_segment:
            if not self.is_valid_cell(pos):
                # Obstacle detected, replan from current position to next valid point
                print(f"Obstacle detected at {pos}, replanning local path")
                
                # Find next valid goal in global path
                next_goal = None
                for i in range(closest_idx + 1, len(self.global_path)):
                    if self.is_valid_cell(self.global_path[i]):
                        next_goal = self.global_path[i]
                        break
                
                if next_goal:
                    # Replan local segment
                    replanned = self.plan_global_path(current_position, next_goal)
                    if replanned:
                        self.local_path = replanned[:lookahead_distance]
                        return self.local_path
                
                # If replanning failed, return empty path
                self.local_path = []
                return []
        
        self.local_path = local_segment
        return self.local_path
    
    def grid_to_world(self, grid_pos: Tuple[int, int]) -> Tuple[float, float]:
        """
        Convert grid coordinates to world coordinates.
        
        Args:
            grid_pos: (row, col) in grid
            
        Returns:
            (x, y) in world frame
        """
        row, col = grid_pos
        x = self.grid_origin[0] + col * self.grid_resolution
        y = self.grid_origin[1] + (self.grid_height - row) * self.grid_resolution
        return (x, y)
    
    def world_to_grid(self, world_pos: Tuple[float, float]) -> Tuple[int, int]:
        """
        Convert world coordinates to grid coordinates.
        
        Args:
            world_pos: (x, y) in world frame
            
        Returns:
            (row, col) in grid
        """
        x, y = world_pos
        col = int((x - self.grid_origin[0]) / self.grid_resolution)
        row = self.grid_height - int((y - self.grid_origin[1]) / self.grid_resolution)
        return (row, col)
