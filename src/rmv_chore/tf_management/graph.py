import time
from collections import defaultdict
from typing import Dict, Optional, List
from geometry_msgs.msg import  Transform, TransformStamped
from threading import Thread, RLock
import tf_transformations as tf

from tf_management.transform import TransformRMV, TransformUtils


class FrameDrawingInfo:
    def __init__(self, frame: str, transform: Transform, start_connection: Transform, end_connection: Transform, opacity: float):
        """Information for drawing a TF."""
        self.name = frame
        self.transform = transform
        self.opacity = opacity
        self.start_connection = start_connection
        self.end_connection = end_connection


class Graph(Thread):
    def __init__(self):
        """
        Represents a graph of frames connected by transforms.
        """
        super().__init__()
        self._nodes = set()
        self._edges: Dict[str, Dict[str, TransformRMV]] = defaultdict(dict)
        self._lock = RLock()
        self._running = True

    def run(self):
        """
        Periodically cleans up expired edges in the graph.
        """
        while self._running:
            with self._lock:
                self._removeExpiredEdges()
            time.sleep(0.1)

    def stop(self):
        """Stop the background thread."""
        self._running = False
        self.join()

    def addEdgeFromTransformStamped(self, transformStamped: TransformStamped, static: bool = False, expiration: float = 0):
        """
        Add a transform to the graph based on a TransformStamped message.

        Args:
            transformStamped (TransformStamped): The transform message.
            static (bool): Whether the transform is static or not.
            expiration (float): The expiration duration in seconds.
        """
        with self._lock:
            header_frame_id = transformStamped.header.frame_id
            child_frame_id = transformStamped.child_frame_id
            transform = transformStamped.transform

            self._nodes.add(header_frame_id)
            self._nodes.add(child_frame_id)

            # Add the forward transform
            transform_rmv = TransformRMV(header_frame_id, child_frame_id, transform)
            transform_rmv.setStatic(static)
            transform_rmv.setExpirationDuration(expiration)
            transform_rmv.setInitialDirection(True)
            self._edges[header_frame_id][child_frame_id] = transform_rmv

            # Add the inverse transform
            inverse_transform = TransformUtils.invertTransform(transform)
            inverse_transform_rmv = TransformRMV(child_frame_id, header_frame_id, inverse_transform)
            inverse_transform_rmv.setStatic(static)
            inverse_transform_rmv.setExpirationDuration(expiration)
            inverse_transform_rmv.setInitialDirection(False)
            self._edges[child_frame_id][header_frame_id] = inverse_transform_rmv

    def _removeExpiredEdges(self):
        """Remove all expired transforms from the graph."""
        for parent, children in list(self._edges.items()):
            for child, transform in list(children.items()):
                if transform.isExpired():
                    del self._edges[parent][child]
            if not self._edges[parent]:
                del self._edges[parent]

    def calculateTransform(self, start: str, end: str) -> Optional[FrameDrawingInfo]:
        """
        Calculate the transform between two frames by traversing the graph.

        Args:
            start (str): The starting frame.
            end (str): The target frame.

        Returns:
            FrameDrawingInfo: Information about the transform if found, otherwise None.
        """
        if start == end:
            return FrameDrawingInfo(start, Transform(), None, None, 1.0)

        path = self._getShortestPath(start, end)
        if not path:
            return None

        current_transform = None
        min_opacity = 1.0
        start_connection = None
        end_connection = None

        for i in range(len(path) - 1):
            parent, child = path[i], path[i + 1]
            edge = self._edges[parent][child]
            transform = edge.getTransform()

            min_opacity = min(min_opacity, edge.getOpacity())

            if current_transform is None:
                current_transform = transform
            else:
                current_transform = TransformUtils.combineTransforms(current_transform, transform)

        if edge._initial_direction:
            inverse_transform = self.getTransform(child, parent)
            start_connection = TransformUtils.combineTransforms(current_transform, inverse_transform)
            end_connection = current_transform
        else:
            inverse_transform = self.getTransform(child, parent)
            end_connection = TransformUtils.combineTransforms(current_transform, inverse_transform)
            start_connection = current_transform

        return FrameDrawingInfo(end, current_transform, start_connection, end_connection, min_opacity)

    def _getShortestPath(self, start: str, end: str) -> Optional[List[str]]:
        """
        Find the shortest path between two frames using BFS.

        Args:
            start (str): The starting frame.
            end (str): The target frame.

        Returns:
            List[str]: A list of frames representing the path, or None if no path exists.
        """
        if start not in self._nodes or end not in self._nodes:
            return None

        visited = set()
        queue = [(start, [start])]

        while queue:
            current, path = queue.pop(0)
            if current == end:
                return path

            if current not in visited:
                visited.add(current)
                for neighbor in self._edges.get(current, {}):
                    if neighbor not in visited:
                        queue.append((neighbor, path + [neighbor]))

        return None

    def getAllFrames(self) -> List[str]:
        """Retrieve all frames in the graph."""
        with self._lock:
            return sorted(self._nodes)

    def getTransform(self, parent: str, child: str) -> Optional[Transform]:
        """Retrieve the transform between two frames."""
        with self._lock:
            edge = self._edges.get(parent, {}).get(child, None)
            if edge:
                return edge.getTransform()
            return None
