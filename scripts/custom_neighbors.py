"""Neighbor utilities adapted from peakyquest/Path-Planning-ROS unit2_pp scripts."""

from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple


def _is_cell_free(
    occ_value: int,
    obstacle_threshold: int,
    allow_unknown: bool,
) -> bool:
    if occ_value < 0:
        return allow_unknown
    return occ_value < obstacle_threshold


def find_neighbors(
    index: int,
    width: int,
    height: int,
    costmap: Sequence[int],
    resolution: float,
    obstacle_threshold: int = 50,
    allow_unknown: bool = False,
    allow_diagonal: bool = True,
    cell_cost_weight: float = 2.0,
) -> List[Tuple[int, float]]:
    """Return valid 4/8-connected neighbors as (index, traversal_cost)."""
    x = index % width
    y = index // width

    steps = [
        (-1, 0, resolution),
        (1, 0, resolution),
        (0, -1, resolution),
        (0, 1, resolution),
    ]
    if allow_diagonal:
        diag = resolution * math.sqrt(2.0)
        steps.extend(
            [
                (-1, -1, diag),
                (1, -1, diag),
                (-1, 1, diag),
                (1, 1, diag),
            ]
        )

    neighbors: List[Tuple[int, float]] = []
    for dx, dy, base_cost in steps:
        nx = x + dx
        ny = y + dy
        if nx < 0 or ny < 0 or nx >= width or ny >= height:
            continue

        nidx = ny * width + nx
        occ = int(costmap[nidx])
        if not _is_cell_free(occ, obstacle_threshold, allow_unknown):
            continue

        occ_penalty = 0.0
        if occ >= 0:
            occ_penalty = (occ / 100.0) * cell_cost_weight
        elif allow_unknown:
            occ_penalty = 1.0 * cell_cost_weight

        neighbors.append((nidx, base_cost * (1.0 + occ_penalty)))

    return neighbors
