"""
A* Pathfinding Algorithm Engine
================================
An independent, reusable pathfinding engine implementing the A* Search Algorithm
and Dijkstra's Algorithm (as A* with zero heuristic) on a uniform-cost 2D grid.

Evaluation Function:
    f(n) = g(n) + h(n)
where:
    - g(n) is the exact movement cost from start to node n.
    - h(n) is the estimated remaining cost to the goal (Manhattan distance).
    - f(n) is the estimated total cost through node n.

Movement Rules:
    - 4-directional orthogonal movement: Up, Down, Left, Right.
    - Uniform movement cost of 1 per step.
    - Manhattan distance heuristic: h((r1, c1), (r2, c2)) = |r1 - r2| + |c1 - c2|.
"""

from __future__ import annotations

import heapq
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Iterable, List, Optional, Set, Tuple

Coordinate = Tuple[int, int]


class AlgorithmMode(str, Enum):
    """Pathfinding algorithm selection."""
    ASTAR = "A*"
    DIJKSTRA = "Dijkstra"


class SearchStatus(str, Enum):
    """Status outcomes for a search query."""
    SUCCESS = "PATH_FOUND"
    START_IS_GOAL = "START_IS_GOAL"
    NO_PATH = "NO_PATH_FOUND"
    START_BLOCKED = "START_BLOCKED"
    GOAL_BLOCKED = "GOAL_BLOCKED"
    OUT_OF_BOUNDS = "OUT_OF_BOUNDS"
    INVALID_GRID = "INVALID_GRID"


@dataclass(frozen=True)
class SearchEvent:
    """Represents a discrete event during search exploration for visualization."""
    event_type: str  # "VISIT", "DISCOVER", "PATH", "FAIL"
    coord: Coordinate
    f: float = 0.0
    g: float = 0.0
    h: float = 0.0
    parent: Optional[Coordinate] = None


@dataclass
class SearchResult:
    """Contains comprehensive metrics and results of a pathfinding run."""
    found: bool
    path: List[Coordinate] = field(default_factory=list)
    path_cost: int = 0
    path_length: int = 0
    explored_nodes: List[Coordinate] = field(default_factory=list)
    explored_count: int = 0
    discovered_count: int = 0
    execution_time_ms: float = 0.0
    status: SearchStatus = SearchStatus.NO_PATH
    status_message: str = ""
    trace: List[SearchEvent] = field(default_factory=list)


def manhattan_distance(a: Coordinate, b: Coordinate) -> int:
    """
    Calculate Manhattan distance (L1 norm) between two grid points.

    Formula:
        h((r1, c1), (r2, c2)) = |r1 - r2| + |c1 - c2|

    In a 4-directional uniform-cost grid, Manhattan distance is an admissible
    (never overestimates) and consistent (monotonic) heuristic, ensuring
    optimality of the A* search algorithm.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def zero_heuristic(a: Coordinate, b: Coordinate) -> int:
    """
    Zero heuristic function (h(n) = 0).

    Reduces A* evaluation function f(n) = g(n) + 0 = g(n), which is mathematically
    identical to Dijkstra's algorithm with uniform step costs.
    """
    return 0


def reconstruct_path(
    parents: dict[Coordinate, Optional[Coordinate]],
    current: Coordinate
) -> List[Coordinate]:
    """
    Reconstruct the path from goal back to start using the parent pointers.
    Returns path in start-to-goal order.
    """
    path: List[Coordinate] = [current]
    while parents.get(current) is not None:
        parent = parents[current]
        assert parent is not None
        path.append(parent)
        current = parent
    path.reverse()
    return path


def astar_search(
    start: Coordinate,
    goal: Coordinate,
    rows: int,
    cols: int,
    obstacles: Set[Coordinate],
    mode: AlgorithmMode = AlgorithmMode.ASTAR,
    record_trace: bool = True
) -> SearchResult:
    """
    Perform A* or Dijkstra pathfinding on a 2D uniform-cost grid.

    Args:
        start: (row, col) coordinate of starting cell.
        goal: (row, col) coordinate of destination cell.
        rows: Total number of rows in the grid.
        cols: Total number of columns in the grid.
        obstacles: Set of blocked (row, col) coordinates.
        mode: AlgorithmMode.ASTAR (Manhattan heuristic) or AlgorithmMode.DIJKSTRA (h=0).
        record_trace: If True, records chronological exploration steps for visual replay.

    Returns:
        SearchResult dataclass containing path, costs, exploration sequence, and timing.
    """
    start_time = time.perf_counter()

    # 1. Validation checks
    if rows <= 0 or cols <= 0:
        elapsed = (time.perf_counter() - start_time) * 1000.0
        return SearchResult(
            found=False,
            execution_time_ms=elapsed,
            status=SearchStatus.INVALID_GRID,
            status_message=f"Invalid grid dimensions: {rows}x{cols}."
        )

    def in_bounds(coord: Coordinate) -> bool:
        r, c = coord
        return 0 <= r < rows and 0 <= c < cols

    if not in_bounds(start) or not in_bounds(goal):
        elapsed = (time.perf_counter() - start_time) * 1000.0
        return SearchResult(
            found=False,
            execution_time_ms=elapsed,
            status=SearchStatus.OUT_OF_BOUNDS,
            status_message=f"Start {start} or Goal {goal} is out of bounds."
        )

    if start in obstacles:
        elapsed = (time.perf_counter() - start_time) * 1000.0
        return SearchResult(
            found=False,
            execution_time_ms=elapsed,
            status=SearchStatus.START_BLOCKED,
            status_message=f"Start node {start} is an obstacle."
        )

    if goal in obstacles:
        elapsed = (time.perf_counter() - start_time) * 1000.0
        return SearchResult(
            found=False,
            execution_time_ms=elapsed,
            status=SearchStatus.GOAL_BLOCKED,
            status_message=f"Goal node {goal} is an obstacle."
        )

    # 2. Handle start == goal
    if start == goal:
        elapsed = (time.perf_counter() - start_time) * 1000.0
        trace = [SearchEvent("VISIT", start, 0, 0, 0, None)] if record_trace else []
        return SearchResult(
            found=True,
            path=[start],
            path_cost=0,
            path_length=0,
            explored_nodes=[start],
            explored_count=1,
            discovered_count=1,
            execution_time_ms=elapsed,
            status=SearchStatus.START_IS_GOAL,
            status_message="Start position is already the destination.",
            trace=trace
        )

    # 3. Choose heuristic function
    heuristic: Callable[[Coordinate, Coordinate], int]
    if mode == AlgorithmMode.DIJKSTRA:
        heuristic = zero_heuristic
    else:
        heuristic = manhattan_distance

    # 4. Data structures
    # Priority Queue elements: (f_score, h_score, tie_breaker, node)
    # Tie-breaking on h_score directs search toward goal when f_scores match.
    # Tie-breaker counter avoids comparing coordinate tuples directly.
    tie_breaker = 0
    start_h = heuristic(start, goal)
    start_f = start_h
    open_heap: List[Tuple[float, float, int, Coordinate]] = []
    heapq.heappush(open_heap, (start_f, start_h, tie_breaker, start))

    g_score: dict[Coordinate, float] = {start: 0.0}
    parents: dict[Coordinate, Optional[Coordinate]] = {start: None}
    closed_set: Set[Coordinate] = set()
    discovered_set: Set[Coordinate] = {start}
    explored_order: List[Coordinate] = []
    trace: List[SearchEvent] = []

    if record_trace:
        trace.append(SearchEvent("DISCOVER", start, start_f, 0.0, start_h, None))

    # 4 orthogonal movement directions: Up, Down, Left, Right
    DIRECTIONS: Tuple[Tuple[int, int], ...] = ((-1, 0), (1, 0), (0, -1), (0, 1))

    # 5. Search loop
    goal_reached = False
    while open_heap:
        current_f, current_h, _, current = heapq.heappop(open_heap)

        # Avoid redundant processing if node is already expanded
        if current in closed_set:
            continue

        # Close current node
        closed_set.add(current)
        explored_order.append(current)
        current_g = g_score[current]

        if record_trace:
            trace.append(SearchEvent(
                "VISIT", current, current_f, current_g, current_h, parents.get(current)
            ))

        # Check if destination reached
        if current == goal:
            goal_reached = True
            break

        # Expand 4 orthogonal neighbors
        curr_r, curr_c = current
        for dr, dc in DIRECTIONS:
            neighbor: Coordinate = (curr_r + dr, curr_c + dc)

            # Skip out-of-bounds or obstacle cells
            if not in_bounds(neighbor) or neighbor in obstacles:
                continue

            # Skip already closed nodes (since uniform cost edge = 1 with admissible heuristic)
            if neighbor in closed_set:
                continue

            tentative_g = current_g + 1.0

            # If better path discovered to neighbor
            if tentative_g < g_score.get(neighbor, float("inf")):
                g_score[neighbor] = tentative_g
                parents[neighbor] = current
                h_val = heuristic(neighbor, goal)
                f_val = tentative_g + h_val

                tie_breaker += 1
                heapq.heappush(open_heap, (f_val, h_val, tie_breaker, neighbor))
                discovered_set.add(neighbor)

                if record_trace:
                    trace.append(SearchEvent(
                        "DISCOVER", neighbor, f_val, tentative_g, h_val, current
                    ))

    elapsed = (time.perf_counter() - start_time) * 1000.0

    if goal_reached:
        path = reconstruct_path(parents, goal)
        path_len = len(path) - 1
        path_cost = path_len  # Uniform cost of 1 per move

        if record_trace:
            for p in path:
                trace.append(SearchEvent("PATH", p))

        return SearchResult(
            found=True,
            path=path,
            path_cost=path_cost,
            path_length=path_len,
            explored_nodes=explored_order,
            explored_count=len(closed_set),
            discovered_count=len(discovered_set),
            execution_time_ms=elapsed,
            status=SearchStatus.SUCCESS,
            status_message=f"Optimal path found: {path_len} moves (Cost: {path_cost}).",
            trace=trace
        )
    else:
        if record_trace:
            trace.append(SearchEvent("FAIL", goal))

        return SearchResult(
            found=False,
            path=[],
            path_cost=0,
            path_length=0,
            explored_nodes=explored_order,
            explored_count=len(closed_set),
            discovered_count=len(discovered_set),
            execution_time_ms=elapsed,
            status=SearchStatus.NO_PATH,
            status_message="No path exists from start to goal.",
            trace=trace
        )
