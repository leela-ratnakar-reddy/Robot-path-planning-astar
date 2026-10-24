"""
Automated Unit Tests for A* Path Planning
==========================================
Covers the 15 required test scenarios:
1. Direct unobstructed route.
2. Route around a single obstacle.
3. Route through a narrow corridor.
4. Start equal to goal.
5. Unreachable goal.
6. Blocked start.
7. Blocked goal.
8. Out-of-bounds coordinates.
9. Invalid grid dimensions.
10. Repeated searches on the same grid.
11. Searches after obstacles are added or removed.
12. Path adjacency and obstacle avoidance.
13. Correct start and goal positions in the returned path.
14. Shortest-path correctness compared with BFS ground truth.
15. A* versus Dijkstra path-cost equivalence on valid maps.
"""

from __future__ import annotations

import collections
import unittest
from typing import List, Optional, Set, Tuple

from astar import (
    AlgorithmMode,
    Coordinate,
    SearchResult,
    SearchStatus,
    astar_search,
    manhattan_distance,
)
from grid import GridModel


def bfs_shortest_path(
    start: Coordinate,
    goal: Coordinate,
    rows: int,
    cols: int,
    obstacles: Set[Coordinate]
) -> Optional[List[Coordinate]]:
    """
    Independent Breadth-First Search (BFS) reference implementation to provide
    unquestionable ground-truth shortest path cost on uniform-cost 2D grids.
    """
    if start == goal:
        return [start]
    if start in obstacles or goal in obstacles:
        return None

    def in_bounds(coord: Coordinate) -> bool:
        return 0 <= coord[0] < rows and 0 <= coord[1] < cols

    queue = collections.deque([start])
    visited: Set[Coordinate] = {start}
    parents: dict[Coordinate, Optional[Coordinate]] = {start: None}

    while queue:
        curr = queue.popleft()
        if curr == goal:
            path = [curr]
            while parents[curr] is not None:
                curr = parents[curr]  # type: ignore
                path.append(curr)
            path.reverse()
            return path

        r, c = curr
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nbr = (r + dr, c + dc)
            if in_bounds(nbr) and nbr not in obstacles and nbr not in visited:
                visited.add(nbr)
                parents[nbr] = curr
                queue.append(nbr)

    return None


class TestAStarPathfinding(unittest.TestCase):
    """Test suite verifying A* algorithm correctness and resilience."""

    def _verify_valid_path(
        self,
        path: List[Coordinate],
        start: Coordinate,
        goal: Coordinate,
        obstacles: Set[Coordinate]
    ) -> None:
        """Helper to thoroughly validate structural path invariants."""
        self.assertGreater(len(path), 0, "Path must not be empty.")
        self.assertEqual(path[0], start, f"Path must begin at requested start {start}.")
        self.assertEqual(path[-1], goal, f"Path must terminate at requested goal {goal}.")

        for i in range(len(path)):
            node = path[i]
            # No path node may lie on an obstacle
            self.assertNotIn(node, obstacles, f"Path cell {node} is on an obstacle!")
            if i > 0:
                prev = path[i - 1]
                # Consecutive cells must be strictly orthogonally adjacent (Manhattan distance == 1)
                dist = manhattan_distance(prev, node)
                self.assertEqual(
                    dist, 1,
                    f"Path steps ({prev} -> {node}) must be orthogonally adjacent (distance 1, got {dist})."
                )

    # 1. A direct unobstructed route
    def test_direct_unobstructed_route(self) -> None:
        start = (2, 2)
        goal = (2, 8)
        res = astar_search(start, goal, rows=10, cols=10, obstacles=set())
        self.assertTrue(res.found)
        self.assertEqual(res.status, SearchStatus.SUCCESS)
        self.assertEqual(res.path_length, 6)
        self.assertEqual(res.path_cost, 6)
        self._verify_valid_path(res.path, start, goal, set())

    # 2. A route around a single obstacle
    def test_route_around_single_obstacle(self) -> None:
        start = (5, 3)
        goal = (5, 5)
        # Block the direct path node at (5, 4)
        obstacles = {(5, 4)}
        res = astar_search(start, goal, rows=10, cols=10, obstacles=obstacles)
        self.assertTrue(res.found)
        self.assertEqual(res.path_length, 4)  # Moves around the obstacle: (5,3)->(4,3)->(4,4)->(4,5)->(5,5) or downward
        self.assertEqual(res.path_cost, 4)
        self.assertNotIn((5, 4), res.path)
        self._verify_valid_path(res.path, start, goal, obstacles)

    # 3. A route through a narrow corridor
    def test_route_through_narrow_corridor(self) -> None:
        # Build an enclosed tunnel from (1, 1) to (1, 6)
        rows, cols = 5, 8
        start = (2, 1)
        goal = (2, 6)
        obstacles: Set[Coordinate] = set()
        # Wall above tunnel (row 1) and wall below tunnel (row 3)
        for c in range(1, 7):
            obstacles.add((1, c))
            obstacles.add((3, c))

        res = astar_search(start, goal, rows=rows, cols=cols, obstacles=obstacles)
        self.assertTrue(res.found)
        self.assertEqual(res.path_length, 5)
        expected_path = [(2, 1), (2, 2), (2, 3), (2, 4), (2, 5), (2, 6)]
        self.assertEqual(res.path, expected_path)
        self._verify_valid_path(res.path, start, goal, obstacles)

    # 4. Start equal to goal
    def test_start_equal_to_goal(self) -> None:
        coord = (4, 4)
        res = astar_search(coord, coord, rows=10, cols=10, obstacles=set())
        self.assertTrue(res.found)
        self.assertEqual(res.status, SearchStatus.START_IS_GOAL)
        self.assertEqual(res.path, [coord])
        self.assertEqual(res.path_length, 0)
        self.assertEqual(res.path_cost, 0)

    # 5. An unreachable goal
    def test_unreachable_goal(self) -> None:
        start = (0, 0)
        goal = (5, 5)
        # Completely surround goal with walls
        obstacles = {
            (4, 4), (4, 5), (4, 6),
            (5, 4),         (5, 6),
            (6, 4), (6, 5), (6, 6)
        }
        res = astar_search(start, goal, rows=10, cols=10, obstacles=obstacles)
        self.assertFalse(res.found)
        self.assertEqual(res.status, SearchStatus.NO_PATH)
        self.assertEqual(len(res.path), 0)
        self.assertGreater(res.explored_count, 0)

    # 6. Blocked start
    def test_blocked_start(self) -> None:
        start = (3, 3)
        goal = (7, 7)
        obstacles = {start}
        res = astar_search(start, goal, rows=10, cols=10, obstacles=obstacles)
        self.assertFalse(res.found)
        self.assertEqual(res.status, SearchStatus.START_BLOCKED)
        self.assertEqual(len(res.path), 0)

    # 7. Blocked goal
    def test_blocked_goal(self) -> None:
        start = (3, 3)
        goal = (7, 7)
        obstacles = {goal}
        res = astar_search(start, goal, rows=10, cols=10, obstacles=obstacles)
        self.assertFalse(res.found)
        self.assertEqual(res.status, SearchStatus.GOAL_BLOCKED)
        self.assertEqual(len(res.path), 0)

    # 8. Out-of-bounds coordinates
    def test_out_of_bounds_coordinates(self) -> None:
        rows, cols = 10, 10
        # Negative start
        res1 = astar_search((-1, 2), (5, 5), rows, cols, set())
        self.assertFalse(res1.found)
        self.assertEqual(res1.status, SearchStatus.OUT_OF_BOUNDS)

        # Start beyond grid
        res2 = astar_search((10, 2), (5, 5), rows, cols, set())
        self.assertFalse(res2.found)
        self.assertEqual(res2.status, SearchStatus.OUT_OF_BOUNDS)

        # Goal beyond grid
        res3 = astar_search((2, 2), (5, 12), rows, cols, set())
        self.assertFalse(res3.found)
        self.assertEqual(res3.status, SearchStatus.OUT_OF_BOUNDS)

    # 9. Invalid grid dimensions
    def test_invalid_grid_dimensions(self) -> None:
        res1 = astar_search((0, 0), (0, 0), rows=0, cols=10, obstacles=set())
        self.assertFalse(res1.found)
        self.assertEqual(res1.status, SearchStatus.INVALID_GRID)

        res2 = astar_search((0, 0), (0, 0), rows=5, cols=-3, obstacles=set())
        self.assertFalse(res2.found)
        self.assertEqual(res2.status, SearchStatus.INVALID_GRID)

        # Also test GridModel raises ValueError
        with self.assertRaises(ValueError):
            GridModel(rows=0, cols=10)
        with self.assertRaises(ValueError):
            GridModel(rows=-5, cols=-5)

    # 10. Repeated searches on the same grid
    def test_repeated_searches_same_grid(self) -> None:
        start = (1, 1)
        goal = (8, 8)
        obstacles = {(4, 4), (4, 5), (5, 4), (5, 5)}
        res1 = astar_search(start, goal, 10, 10, obstacles)
        res2 = astar_search(start, goal, 10, 10, obstacles)
        res3 = astar_search(start, goal, 10, 10, obstacles)

        self.assertTrue(res1.found)
        self.assertTrue(res2.found)
        self.assertTrue(res3.found)
        self.assertEqual(res1.path, res2.path)
        self.assertEqual(res2.path, res3.path)
        self.assertEqual(res1.path_cost, res2.path_cost)
        self.assertEqual(res1.explored_count, res2.explored_count)

    # 11. Searches after obstacles are added or removed
    def test_searches_after_obstacles_modified(self) -> None:
        grid = GridModel(rows=10, cols=10, start=(2, 2), goal=(2, 6))

        # Initial search: empty grid
        res_empty = astar_search(grid.start, grid.goal, grid.rows, grid.cols, grid.obstacles)
        self.assertEqual(res_empty.path_length, 4)

        # Add vertical barrier blocking row 2
        grid.add_obstacle(1, 4)
        grid.add_obstacle(2, 4)
        grid.add_obstacle(3, 4)

        res_blocked = astar_search(grid.start, grid.goal, grid.rows, grid.cols, grid.obstacles)
        self.assertTrue(res_blocked.found)
        # Must detour around the 3-block wall (cost increases from 4 to 6)
        self.assertGreater(res_blocked.path_length, res_empty.path_length)
        self._verify_valid_path(res_blocked.path, grid.start, grid.goal, grid.obstacles)

        # Remove barrier
        grid.remove_obstacle(2, 4)
        res_cleared = astar_search(grid.start, grid.goal, grid.rows, grid.cols, grid.obstacles)
        self.assertEqual(res_cleared.path_length, 4)
        self.assertEqual(res_cleared.path, res_empty.path)

    # 12. Path adjacency and obstacle avoidance
    def test_path_adjacency_and_obstacle_avoidance(self) -> None:
        grid = GridModel(rows=15, cols=20)
        grid.generate_random_obstacles(density=0.25, seed=123)
        res = astar_search(grid.start, grid.goal, grid.rows, grid.cols, grid.obstacles)
        if res.found:
            self._verify_valid_path(res.path, grid.start, grid.goal, grid.obstacles)

    # 13. Correct start and goal positions in the returned path
    def test_correct_start_and_goal_positions(self) -> None:
        start = (3, 7)
        goal = (11, 2)
        res = astar_search(start, goal, 15, 15, obstacles=set())
        self.assertTrue(res.found)
        self.assertEqual(res.path[0], start)
        self.assertEqual(res.path[-1], goal)

    # 14. Shortest-path correctness compared with BFS ground truth
    def test_shortest_path_optimality_vs_bfs(self) -> None:
        # Generate multiple maze configurations and verify A* matches BFS exactly
        for seed in [10, 25, 99, 137]:
            grid = GridModel(rows=12, cols=12)
            grid.generate_random_obstacles(density=0.20, seed=seed)

            bfs_path = bfs_shortest_path(grid.start, grid.goal, grid.rows, grid.cols, grid.obstacles)
            astar_res = astar_search(grid.start, grid.goal, grid.rows, grid.cols, grid.obstacles)

            if bfs_path is None:
                self.assertFalse(astar_res.found, f"BFS found no path for seed {seed}, but A* found one.")
            else:
                self.assertTrue(astar_res.found, f"BFS found path for seed {seed}, but A* failed.")
                bfs_cost = len(bfs_path) - 1
                self.assertEqual(
                    astar_res.path_cost, bfs_cost,
                    f"A* cost ({astar_res.path_cost}) must equal optimal BFS cost ({bfs_cost}) for seed {seed}."
                )

    # 15. A* versus Dijkstra path-cost equivalence on the same valid maps
    def test_astar_vs_dijkstra_equivalence(self) -> None:
        for seed in [1, 42, 777]:
            grid = GridModel(rows=15, cols=15)
            grid.generate_random_obstacles(density=0.22, seed=seed)

            res_astar = astar_search(
                grid.start, grid.goal, grid.rows, grid.cols, grid.obstacles,
                mode=AlgorithmMode.ASTAR
            )
            res_dijkstra = astar_search(
                grid.start, grid.goal, grid.rows, grid.cols, grid.obstacles,
                mode=AlgorithmMode.DIJKSTRA
            )

            # Both must agree on reachability
            self.assertEqual(res_astar.found, res_dijkstra.found)

            if res_astar.found:
                # Both must return identical optimal path cost
                self.assertEqual(
                    res_astar.path_cost, res_dijkstra.path_cost,
                    f"A* path cost {res_astar.path_cost} must equal Dijkstra {res_dijkstra.path_cost}."
                )
                # A* with Manhattan distance must explore less than or equal to Dijkstra (which expands uniformly)
                self.assertLessEqual(
                    res_astar.explored_count, res_dijkstra.explored_count,
                    f"A* explored ({res_astar.explored_count}) should be <= Dijkstra explored ({res_dijkstra.explored_count})."
                )


if __name__ == "__main__":
    unittest.main()
