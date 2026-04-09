"""A*/Dijkstra planner algorithms adapted from peakyquest/Path-Planning-ROS unit2_pp."""

from __future__ import annotations

import heapq
import math
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Set, Tuple

from custom_neighbors import find_neighbors

NodeCallback = Optional[Callable[[int], None]]


def _euclidean_distance(idx: int, goal_idx: int, width: int, resolution: float) -> float:
    ix = idx % width
    iy = idx // width
    gx = goal_idx % width
    gy = goal_idx // width
    return math.hypot(ix - gx, iy - gy) * resolution


def _reconstruct_path(
    parents: Dict[int, int],
    start_idx: int,
    goal_idx: int,
) -> List[int]:
    path = [goal_idx]
    node = goal_idx
    while node != start_idx:
        node = parents[node]
        path.append(node)
    path.reverse()
    return path


def _step_direction(from_idx: int, to_idx: int, width: int) -> Tuple[int, int]:
    fx = from_idx % width
    fy = from_idx // width
    tx = to_idx % width
    ty = to_idx // width
    return (tx - fx, ty - fy)


def _turn_penalty_cost(
    prev_idx: Optional[int],
    current_idx: int,
    next_idx: int,
    width: int,
    turn_penalty: float,
) -> float:
    if prev_idx is None or turn_penalty <= 0.0:
        return 0.0

    prev_dx, prev_dy = _step_direction(prev_idx, current_idx, width)
    next_dx, next_dy = _step_direction(current_idx, next_idx, width)
    if prev_dx == next_dx and prev_dy == next_dy:
        return 0.0

    prev_norm = math.hypot(prev_dx, prev_dy)
    next_norm = math.hypot(next_dx, next_dy)
    if prev_norm == 0.0 or next_norm == 0.0:
        return 0.0

    cos_theta = (prev_dx * next_dx + prev_dy * next_dy) / (prev_norm * next_norm)
    cos_theta = max(-1.0, min(1.0, cos_theta))
    angle = math.acos(cos_theta)
    return turn_penalty * (angle / math.pi)


def dijkstra(
    start_index: int,
    goal_index: int,
    width: int,
    height: int,
    costmap: Sequence[int],
    resolution: float,
    obstacle_threshold: int = 50,
    allow_unknown: bool = False,
    allow_diagonal: bool = True,
    cell_cost_weight: float = 2.0,
    turn_penalty: float = 0.0,
    on_closed: NodeCallback = None,
    on_frontier: NodeCallback = None,
) -> Tuple[List[int], Set[int], Set[int]]:
    """Compute shortest path using Dijkstra."""
    open_heap: List[Tuple[float, int]] = [(0.0, start_index)]
    g_costs: Dict[int, float] = {start_index: 0.0}
    parents: Dict[int, int] = {}
    closed: Set[int] = set()
    frontier: Set[int] = {start_index}

    while open_heap:
        current_cost, current = heapq.heappop(open_heap)
        if current in closed:
            continue

        frontier.discard(current)
        closed.add(current)
        if on_closed is not None:
            on_closed(current)

        if current == goal_index:
            return _reconstruct_path(parents, start_index, goal_index), closed, frontier

        neighbors = find_neighbors(
            current,
            width,
            height,
            costmap,
            resolution,
            obstacle_threshold=obstacle_threshold,
            allow_unknown=allow_unknown,
            allow_diagonal=allow_diagonal,
            cell_cost_weight=cell_cost_weight,
        )
        parent = parents.get(current)
        for neighbor, step_cost in neighbors:
            if neighbor in closed:
                continue

            tentative = current_cost + step_cost + _turn_penalty_cost(
                parent, current, neighbor, width, turn_penalty
            )
            if tentative >= g_costs.get(neighbor, float('inf')):
                continue

            parents[neighbor] = current
            g_costs[neighbor] = tentative
            heapq.heappush(open_heap, (tentative, neighbor))

            if neighbor not in frontier:
                frontier.add(neighbor)
                if on_frontier is not None:
                    on_frontier(neighbor)

    return [], closed, frontier


def a_star(
    start_index: int,
    goal_index: int,
    width: int,
    height: int,
    costmap: Sequence[int],
    resolution: float,
    obstacle_threshold: int = 50,
    allow_unknown: bool = False,
    allow_diagonal: bool = True,
    cell_cost_weight: float = 2.0,
    turn_penalty: float = 0.0,
    on_closed: NodeCallback = None,
    on_frontier: NodeCallback = None,
) -> Tuple[List[int], Set[int], Set[int]]:
    """Compute shortest path using A* (Euclidean heuristic)."""
    start_h = _euclidean_distance(start_index, goal_index, width, resolution)
    open_heap: List[Tuple[float, float, int]] = [(start_h, 0.0, start_index)]
    g_costs: Dict[int, float] = {start_index: 0.0}
    parents: Dict[int, int] = {}
    closed: Set[int] = set()
    frontier: Set[int] = {start_index}

    while open_heap:
        _, current_g, current = heapq.heappop(open_heap)
        if current in closed:
            continue

        frontier.discard(current)
        closed.add(current)
        if on_closed is not None:
            on_closed(current)

        if current == goal_index:
            return _reconstruct_path(parents, start_index, goal_index), closed, frontier

        neighbors = find_neighbors(
            current,
            width,
            height,
            costmap,
            resolution,
            obstacle_threshold=obstacle_threshold,
            allow_unknown=allow_unknown,
            allow_diagonal=allow_diagonal,
            cell_cost_weight=cell_cost_weight,
        )
        parent = parents.get(current)
        for neighbor, step_cost in neighbors:
            if neighbor in closed:
                continue

            tentative_g = current_g + step_cost + _turn_penalty_cost(
                parent, current, neighbor, width, turn_penalty
            )
            if tentative_g >= g_costs.get(neighbor, float('inf')):
                continue

            parents[neighbor] = current
            g_costs[neighbor] = tentative_g
            h = _euclidean_distance(neighbor, goal_index, width, resolution)
            heapq.heappush(open_heap, (tentative_g + h, tentative_g, neighbor))

            if neighbor not in frontier:
                frontier.add(neighbor)
                if on_frontier is not None:
                    on_frontier(neighbor)

    return [], closed, frontier
