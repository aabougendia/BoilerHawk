"""
Path planning utilities.
Implements:
  - A* algorithm for global path planning (PathPlanner)
  - Vector Field Histogram for reactive obstacle avoidance (VFHPlanner)
"""

import math
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


# =====================================================================
#  VFH (Vector Field Histogram) Planner
# =====================================================================

class VFHPlanner:
    """
    Reactive obstacle avoidance using the Vector Field Histogram algorithm.

    Works on a robot-centric occupancy grid (drone at centre).  Always
    produces a steering direction — never returns "no path found".
    """

    def __init__(
        self,
        occupancy_threshold: int = 50,
        safety_radius_cells: int = 3,
        num_sectors: int = 72,
        valley_threshold: float = 0.5,
        num_waypoints: int = 3,
        waypoint_spacing: float = 1.0,
    ):
        self.occupancy_threshold = occupancy_threshold
        self.safety_radius_cells = safety_radius_cells
        self.num_sectors = num_sectors
        self.valley_threshold = valley_threshold
        self.num_waypoints = num_waypoints
        self.waypoint_spacing = waypoint_spacing

        self.occupancy_grid: Optional[np.ndarray] = None
        self.grid_resolution: float = 0.1
        self.grid_width: int = 0
        self.grid_height: int = 0
        self.grid_origin: Tuple[float, float] = (0.0, 0.0)

        self._sector_width = 2.0 * math.pi / self.num_sectors

    # ----- grid bookkeeping (same interface as PathPlanner) -----------

    def update_occupancy_grid(
        self,
        grid_data: np.ndarray,
        resolution: float,
        origin: Tuple[float, float],
    ):
        self.occupancy_grid = grid_data
        self.grid_resolution = resolution
        self.grid_height, self.grid_width = grid_data.shape
        self.grid_origin = origin

    def grid_to_world(self, grid_pos: Tuple[int, int]) -> Tuple[float, float]:
        row, col = grid_pos
        x = self.grid_origin[0] + col * self.grid_resolution
        y = self.grid_origin[1] + (self.grid_height - row) * self.grid_resolution
        return (x, y)

    def world_to_grid(self, world_pos: Tuple[float, float]) -> Tuple[int, int]:
        x, y = world_pos
        col = int((x - self.grid_origin[0]) / self.grid_resolution)
        row = self.grid_height - int((y - self.grid_origin[1]) / self.grid_resolution)
        return (row, col)

    # ----- inflation --------------------------------------------------

    def inflate_grid(self, grid: np.ndarray) -> np.ndarray:
        """Binary dilation: grow occupied cells by *safety_radius_cells*."""
        occupied = grid >= self.occupancy_threshold
        if self.safety_radius_cells <= 0:
            return occupied.astype(np.int8) * 100

        r = self.safety_radius_cells
        y_k, x_k = np.ogrid[-r:r + 1, -r:r + 1]
        kernel = (x_k * x_k + y_k * y_k) <= r * r

        h, w = occupied.shape
        inflated = np.zeros_like(occupied)
        occ_rows, occ_cols = np.where(occupied)
        for oi, oj in zip(occ_rows, occ_cols):
            r0 = max(oi - r, 0)
            r1 = min(oi + r + 1, h)
            c0 = max(oj - r, 0)
            c1 = min(oj + r + 1, w)
            kr0 = r0 - (oi - r)
            kr1 = kernel.shape[0] - ((oi + r + 1) - r1)
            kc0 = c0 - (oj - r)
            kc1 = kernel.shape[1] - ((oj + r + 1) - c1)
            inflated[r0:r1, c0:c1] |= kernel[kr0:kr1, kc0:kc1]

        return inflated.astype(np.int8) * 100

    # ----- polar histogram --------------------------------------------

    def build_polar_histogram(self, inflated_grid: np.ndarray) -> np.ndarray:
        """
        Build a polar obstacle-density histogram around the grid centre.

        Returns an array of shape *(num_sectors,)* where each element is the
        cumulative obstacle weight for that angular sector.
        """
        h, w = inflated_grid.shape
        cr, cc = h // 2, w // 2
        max_range_cells = max(h, w) / 2.0

        histogram = np.zeros(self.num_sectors, dtype=np.float64)
        occ_rows, occ_cols = np.where(inflated_grid >= self.occupancy_threshold)
        if len(occ_rows) == 0:
            return histogram

        dr = occ_rows.astype(np.float64) - cr
        dc = occ_cols.astype(np.float64) - cc
        # angle: 0 = +col (ahead / camera +X), positive CCW
        angles = np.arctan2(-dr, dc)  # -dr because row increases downward
        dists = np.sqrt(dr * dr + dc * dc)
        dists = np.maximum(dists, 1.0)

        # Weight: closer obstacles count more (linear falloff)
        a_coeff = 1.0
        b_coeff = a_coeff / max_range_cells
        weights = np.maximum(0.0, a_coeff - b_coeff * dists)

        sector_indices = self._angle_to_sector(angles)
        np.add.at(histogram, sector_indices, weights)

        # Normalise so threshold is scale-independent
        hist_max = histogram.max()
        if hist_max > 0:
            histogram /= hist_max

        return histogram

    # ----- valley / direction selection --------------------------------

    def find_best_direction(
        self, histogram: np.ndarray, preferred_angle: float
    ) -> float:
        """
        Pick the best clear direction closest to *preferred_angle*.

        Returns an angle in radians (robot frame, 0 = ahead, CCW positive).
        """
        free = histogram < self.valley_threshold
        n = self.num_sectors

        if free.all():
            return preferred_angle
        if not free.any():
            # Fully blocked — pick the least-dense sector
            idx = int(np.argmin(histogram))
            return self._sector_to_angle(idx)

        # Find contiguous free runs (valleys), wrapping around
        valleys: List[Tuple[int, int]] = []  # (start_sector, length)
        start = None
        length = 0
        # Double-pass to handle wrap-around
        extended = np.concatenate([free, free])
        for i in range(2 * n):
            if extended[i]:
                if start is None:
                    start = i % n
                length += 1
            else:
                if start is not None and length > 0:
                    valleys.append((start, length))
                start = None
                length = 0
        if start is not None and length > 0:
            valleys.append((start, length))

        # De-duplicate valleys that wrapped (keep the longest per start)
        if not valleys:
            idx = int(np.argmin(histogram))
            return self._sector_to_angle(idx)

        pref_sector = self._angle_to_sector(np.array([preferred_angle]))[0]
        best_angle = preferred_angle
        best_cost = float('inf')

        margin = 2  # sectors of extra angular safety margin near obstacle edges

        for v_start, v_len in valleys:
            v_len = min(v_len, n)  # cap at full circle
            
            # If the preferred angle is within this safe valley, steer directly towards it
            if (pref_sector - v_start) % n < v_len:
                return preferred_angle

            # Otherwise, evaluate the edges (plus margin) and the centre of the valley
            m = min(margin, v_len // 2)
            c1_sector = (v_start + m) % n
            c2_sector = (v_start + v_len - 1 - m) % n
            c3_sector = (v_start + v_len // 2) % n
            
            for cand_sector in [c1_sector, c2_sector, c3_sector]:
                cand_angle = self._sector_to_angle(cand_sector)
                cost = abs(self._angle_diff(cand_angle, preferred_angle))
                if cost < best_cost:
                    best_cost = cost
                    best_angle = cand_angle

        return best_angle

    # ----- end-to-end plan -------------------------------------------

    def plan(
        self,
        preferred_angle_rad: float = 0.0,
        num_waypoints: Optional[int] = None,
        spacing_m: Optional[float] = None,
    ) -> Tuple[List[Tuple[int, int]], float]:
        """
        Run the full VFH pipeline and return a short waypoint list in
        **grid coordinates** (row, col) together with the chosen heading.

        Returns:
            (waypoints, chosen_angle_rad)  — empty list + 0.0 when no grid.
        """
        if self.occupancy_grid is None:
            return [], 0.0

        nw = num_waypoints if num_waypoints is not None else self.num_waypoints
        sp = spacing_m if spacing_m is not None else self.waypoint_spacing

        inflated = self.inflate_grid(self.occupancy_grid)
        histogram = self.build_polar_histogram(inflated)
        chosen_angle = self.find_best_direction(histogram, preferred_angle_rad)

        cr = self.grid_height // 2
        cc = self.grid_width // 2
        spacing_cells = sp / self.grid_resolution

        waypoints: List[Tuple[int, int]] = []
        for i in range(1, nw + 1):
            d = i * spacing_cells
            col = int(round(cc + d * math.cos(chosen_angle)))
            row = int(round(cr - d * math.sin(chosen_angle)))  # -sin: row↓
            row = max(0, min(self.grid_height - 1, row))
            col = max(0, min(self.grid_width - 1, col))
            waypoints.append((row, col))

        return waypoints, chosen_angle

    # ----- helpers ----------------------------------------------------

    def _angle_to_sector(self, angles: np.ndarray) -> np.ndarray:
        """Map angles (rad) to sector indices [0, num_sectors)."""
        a = np.mod(angles, 2.0 * math.pi)
        return (a / self._sector_width).astype(int) % self.num_sectors

    def _sector_to_angle(self, sector: int) -> float:
        """Return the centre angle (rad) of a sector."""
        return (sector + 0.5) * self._sector_width

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Signed shortest angular difference a − b, result in (−pi, pi]."""
        d = (a - b) % (2.0 * math.pi)
        if d > math.pi:
            d -= 2.0 * math.pi
        return d
