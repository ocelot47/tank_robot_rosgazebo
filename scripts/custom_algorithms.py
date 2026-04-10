"""A*/Dijkstra planner algorithms adapted from peakyquest/Path-Planning-ROS unit2_pp."""

from __future__ import annotations

import heapq
import itertools
import math
from typing import Callable, Dict, List, Optional, Sequence, Set, Tuple

from custom_neighbors import find_neighbors

NodeCallback = Optional[Callable[[int], None]]
_SQRT2 = math.sqrt(2.0)


def _normalize_heuristic_mode(mode: str, allow_diagonal: bool) -> str:
    selected_mode = mode.strip().lower() if mode else 'auto'
    if selected_mode == 'auto':
        return 'octile' if allow_diagonal else 'manhattan'
    if selected_mode in ('manhattan', 'octile', 'euclidean'):
        return selected_mode
    return 'euclidean'


def _build_heuristic(
    goal_idx: int,
    width: int,
    resolution: float,
    mode: str,
) -> Callable[[int], float]:
    gx = goal_idx % width
    gy = goal_idx // width
    if mode == 'manhattan':
        def manhattan(idx: int) -> float:
            ix = idx % width
            iy = idx // width
            return (abs(ix - gx) + abs(iy - gy)) * resolution
        return manhattan

    if mode == 'octile':
        d = resolution
        octile_cross_term = (resolution * _SQRT2) - (2.0 * d)

        def octile(idx: int) -> float:
            ix = idx % width
            iy = idx // width
            dx = abs(ix - gx)
            dy = abs(iy - gy)
            return d * (dx + dy) + octile_cross_term * min(dx, dy)
        return octile

    def euclidean(idx: int) -> float:
        ix = idx % width
        iy = idx // width
        return math.hypot(ix - gx, iy - gy) * resolution

    return euclidean


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
    push_order = itertools.count()
    next_push = push_order.__next__
    open_heap: List[Tuple[float, int, int]] = [(0.0, next_push(), start_index)]
    g_costs: Dict[int, float] = {start_index: 0.0}
    parents: Dict[int, int] = {}
    closed: Set[int] = set()
    frontier: Set[int] = {start_index}
    inf = float('inf')
    push_heap = heapq.heappush
    pop_heap = heapq.heappop

    while open_heap:
        current_cost, _, current = pop_heap(open_heap)
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
            if tentative >= g_costs.get(neighbor, inf):
                continue

            parents[neighbor] = current
            g_costs[neighbor] = tentative
            push_heap(open_heap, (tentative, next_push(), neighbor))

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
    heuristic_mode: str = 'auto',
    heuristic_weight: float = 1.0,
    on_closed: NodeCallback = None,
    on_frontier: NodeCallback = None,
) -> Tuple[List[int], Set[int], Set[int]]:
    """Compute shortest path using A* with configurable heuristic."""
    push_order = itertools.count()
    next_push = push_order.__next__
    push_heap = heapq.heappush
    pop_heap = heapq.heappop
    w = max(0.0, float(heuristic_weight))
    resolved_mode = _normalize_heuristic_mode(heuristic_mode, allow_diagonal)
    heuristic = _build_heuristic(
        goal_index,
        width,
        resolution,
        resolved_mode,
    )
    start_h = heuristic(start_index)
    open_heap: List[Tuple[float, float, int, int]] = [
        (w * start_h, start_h, next_push(), start_index)
    ]
    g_costs: Dict[int, float] = {start_index: 0.0}
    parents: Dict[int, int] = {}
    closed: Set[int] = set()
    frontier: Set[int] = {start_index}
    inf = float('inf')

    while open_heap:
        _, _, _, current = pop_heap(open_heap)
        if current in closed:
            continue
        current_g = g_costs[current]

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
            if tentative_g >= g_costs.get(neighbor, inf):
                continue

            parents[neighbor] = current
            g_costs[neighbor] = tentative_g
            h = heuristic(neighbor)
            push_heap(
                open_heap,
                (tentative_g + w * h, h, next_push(), neighbor),
            )

            if neighbor not in frontier:
                frontier.add(neighbor)
                if on_frontier is not None:
                    on_frontier(neighbor)

    return [], closed, frontier
