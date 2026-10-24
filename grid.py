"""
Grid Environment Model
======================
Manages grid dimensions, start and goal endpoints, obstacle distributions,
explored and frontier cell states, the reconstructed path, and the robot position.

Designed to be independent of rendering and visualization code.
"""

from __future__ import annotations

import random
from typing import List, Optional, Set, Tuple

Coordinate = Tuple[int, int]


class GridModel:
    """
    Independent 2D grid representation for robot path planning.

    Attributes:
        rows (int): Number of rows in the grid.
        cols (int): Number of columns in the grid.
        start (Coordinate): (row, col) coordinates of start node.
        goal (Coordinate): (row, col) coordinates of destination node.
        obstacles (Set[Coordinate]): Set of obstacle cells.
        explored (Set[Coordinate]): Explored/closed cells during search.
        frontier (Set[Coordinate]): Discovered/open cells in frontier.
        path (List[Coordinate]): Reconstructed shortest path coordinates.
        robot_pos (Coordinate): Current coordinate of the animated robot.
    """

    def __init__(
        self,
        rows: int = 25,
        cols: int = 35,
        start: Optional[Coordinate] = None,
        goal: Optional[Coordinate] = None
    ) -> None:
        if rows <= 0 or cols <= 0:
            raise ValueError(f"Grid dimensions must be positive integers, got {rows}x{cols}.")

        self.rows = rows
        self.cols = cols

        # Default start: left side, vertically centered
        # Default goal: right side, vertically centered
        self.start: Coordinate = start if start is not None else (rows // 2, 4)
        self.goal: Coordinate = goal if goal is not None else (rows // 2, cols - 5)

        # Validate start and goal are inside bounds
        if not self.in_bounds(*self.start):
            raise ValueError(f"Start coordinate {self.start} out of bounds for {rows}x{cols} grid.")
        if not self.in_bounds(*self.goal):
            raise ValueError(f"Goal coordinate {self.goal} out of bounds for {rows}x{cols} grid.")

        self.obstacles: Set[Coordinate] = set()
        self.explored: Set[Coordinate] = set()
        self.frontier: Set[Coordinate] = set()
        self.path: List[Coordinate] = []
        self.robot_pos: Coordinate = self.start

    def in_bounds(self, r: int, c: int) -> bool:
        """Check if (r, c) is within grid boundaries."""
        return 0 <= r < self.rows and 0 <= c < self.cols

    def is_obstacle(self, r: int, c: int) -> bool:
        """Check if (r, c) is an obstacle."""
        return (r, c) in self.obstacles

    def is_walkable(self, r: int, c: int) -> bool:
        """Check if (r, c) is inside grid and not blocked."""
        return self.in_bounds(r, c) and (r, c) not in self.obstacles

    def get_neighbors(self, r: int, c: int) -> List[Coordinate]:
        """Return all valid 4-directional orthogonal neighbors."""
        neighbors: List[Coordinate] = []
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = r + dr, c + dc
            if self.is_walkable(nr, nc):
                neighbors.append((nr, nc))
        return neighbors

    def set_start(self, r: int, c: int) -> bool:
        """Set a new start position. Returns True if successful."""
        if not self.in_bounds(r, c):
            return False
        if (r, c) in self.obstacles:
            return False
        if (r, c) == self.goal:
            return False
        self.start = (r, c)
        self.robot_pos = (r, c)
        self.clear_search_metadata()
        return True

    def set_goal(self, r: int, c: int) -> bool:
        """Set a new goal position. Returns True if successful."""
        if not self.in_bounds(r, c):
            return False
        if (r, c) in self.obstacles:
            return False
        if (r, c) == self.start:
            return False
        self.goal = (r, c)
        self.clear_search_metadata()
        return True

    def add_obstacle(self, r: int, c: int) -> bool:
        """Add an obstacle at (r, c). Cannot overwrite start or goal."""
        if not self.in_bounds(r, c):
            return False
        if (r, c) == self.start or (r, c) == self.goal:
            return False
        if (r, c) not in self.obstacles:
            self.obstacles.add((r, c))
            self.clear_search_metadata()
            return True
        return False

    def remove_obstacle(self, r: int, c: int) -> bool:
        """Remove an obstacle at (r, c)."""
        if (r, c) in self.obstacles:
            self.obstacles.remove((r, c))
            self.clear_search_metadata()
            return True
        return False

    def toggle_obstacle(self, r: int, c: int) -> bool:
        """Toggle obstacle presence at (r, c)."""
        if (r, c) in self.obstacles:
            return self.remove_obstacle(r, c)
        else:
            return self.add_obstacle(r, c)

    def clear_obstacles(self) -> None:
        """Remove all obstacles and invalidate stale search results."""
        self.obstacles.clear()
        self.clear_search_metadata()

    def clear_search_metadata(self) -> None:
        """Clear explored nodes, frontier nodes, and path while keeping obstacles."""
        self.explored.clear()
        self.frontier.clear()
        self.path.clear()
        self.robot_pos = self.start

    def reset_all(self) -> None:
        """Reset grid completely: clears obstacles and search data."""
        self.clear_obstacles()

    def generate_random_obstacles(
        self,
        density: float = 0.25,
        seed: Optional[int] = None
    ) -> int:
        """
        Generate random obstacles across the grid with a specified density [0.0, 0.7].
        Guarantees that start and goal cells remain unblocked.
        Supports reproducible pseudorandom seed.

        Returns:
            Number of obstacles generated.
        """
        density = max(0.0, min(0.70, density))
        rng = random.Random(seed)
        self.obstacles.clear()
        self.clear_search_metadata()

        for r in range(self.rows):
            for c in range(self.cols):
                if (r, c) == self.start or (r, c) == self.goal:
                    continue
                if rng.random() < density:
                    self.obstacles.add((r, c))

        return len(self.obstacles)

    def load_custom_pattern(self, pattern_type: str = "maze") -> None:
        """
        Create a classic predefined test pattern (e.g. wall, maze, narrow corridor).
        """
        self.clear_obstacles()
        if pattern_type == "wall_with_opening":
            # Vertical wall in the middle with a gap
            mid_col = self.cols // 2
            gap_row = self.rows // 2
            for r in range(self.rows):
                if r != gap_row and r != gap_row + 1:
                    self.obstacles.add((r, mid_col))
        elif pattern_type == "narrow_corridor":
            # S-curve narrow corridor
            mid_row = self.rows // 2
            for c in range(5, self.cols - 5):
                if c != 10 and c != self.cols - 10:
                    self.obstacles.add((mid_row - 2, c))
                    self.obstacles.add((mid_row + 2, c))
        elif pattern_type == "diagonal_slits":
            for r in range(self.rows):
                for c in range(self.cols):
                    if (r + c) % 6 == 0 and (r, c) != self.start and (r, c) != self.goal:
                        self.obstacles.add((r, c))
        self.clear_search_metadata()
