"""PointCloud2 search visualization (ROS2) adapted from unit2_pp/gridviz.py."""

from __future__ import annotations

from typing import Iterable, List, Sequence, Set, Tuple

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def _rgba_u32(r: int, g: int, b: int, a: int = 255) -> int:
    return ((a & 0xFF) << 24) | ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF)


class GridViz:
    def __init__(
        self,
        node: Node,
        frame_id: str,
        resolution: float,
        origin_x: float,
        origin_y: float,
        width: int,
    ) -> None:
        self._node = node
        self._frame_id = frame_id
        self._resolution = resolution
        self._origin_x = origin_x
        self._origin_y = origin_y
        self._width = width

        self._publisher = node.create_publisher(PointCloud2, '/grid_viz', 1)
        self._fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
        ]

        self._colors = {
            'closed': _rgba_u32(255, 235, 120),
            'frontier': _rgba_u32(255, 165, 0),
            'path': _rgba_u32(0, 220, 0),
            'start': _rgba_u32(0, 0, 255),
            'goal': _rgba_u32(255, 0, 0),
        }

    def _index_to_world(self, index: int) -> Tuple[float, float, float]:
        x_cell = index % self._width
        y_cell = index // self._width
        x = self._origin_x + (x_cell + 0.5) * self._resolution
        y = self._origin_y + (y_cell + 0.5) * self._resolution
        return (x, y, 0.03)

    def publish(
        self,
        start_idx: int,
        goal_idx: int,
        path_indices: Sequence[int],
        closed_indices: Set[int],
        frontier_indices: Set[int],
    ) -> None:
        points: List[Tuple[float, float, float, int]] = []

        for idx in closed_indices:
            x, y, z = self._index_to_world(idx)
            points.append((x, y, z, self._colors['closed']))

        for idx in frontier_indices:
            if idx in closed_indices:
                continue
            x, y, z = self._index_to_world(idx)
            points.append((x, y, z, self._colors['frontier']))

        for idx in path_indices:
            x, y, z = self._index_to_world(idx)
            points.append((x, y, z + 0.03, self._colors['path']))

        sx, sy, sz = self._index_to_world(start_idx)
        gx, gy, gz = self._index_to_world(goal_idx)
        points.append((sx, sy, sz + 0.05, self._colors['start']))
        points.append((gx, gy, gz + 0.05, self._colors['goal']))

        header = Header()
        header.frame_id = self._frame_id
        header.stamp = self._node.get_clock().now().to_msg()
        msg = point_cloud2.create_cloud(header, self._fields, points)
        self._publisher.publish(msg)
