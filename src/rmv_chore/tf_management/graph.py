import time
from collections import defaultdict
from typing import Dict, Optional, List, Tuple
from geometry_msgs.msg import TransformStamped, Transform
from threading import Thread, RLock
import tf_transformations as tf


class TransformRMV:
    def __init__(self, parent: str, child: str, transform: Transform, static: bool = False, expiration: float = 2):
        """
        Represents a transform with metadata such as expiration and validity.

        Args:
            parent (str): The parent frame.
            child (str): The child frame.
            transform (Transform): The transform between the parent and child frames.
            static (bool): Whether the transform is static or not.
            expiration (float): The expiration duration of the transform in seconds.
        """
        self._parent = parent
        self._child = child
        self._transform = transform
        self._static = static
        self._expirationDuration = expiration
        self._expirationTime = None if static else time.time() + expiration
        self._validityDuration = 0.2
        self._validityTime = time.time() + self._validityDuration

    def isExpired(self) -> bool:
        """
        Check if the transform has expired.

        Returns:
            bool: True if the transform has expired, False otherwise.
        """
        return not self._static and time.time() > self._expirationTime

    def isValid(self) -> bool:
        """
        Check if the transform is still valid.

        Returns:
            bool: True if the transform is still valid, False otherwise.
        """
        return time.time() < self._validityTime

    def getTransform(self, inverse: bool = False) -> Transform:
        """
        Retrieve the transform, optionally inverted.

        Args:
            inverse (bool): Whether to return the inverse transform.

        Returns:
            Transform: The transform or its inverse.
        """
        if inverse:
            return self._invertTransform(self._transform)
        return self._transform

    def getOpacity(self) -> float:
        """
        Get the opacity of the transform, representing its remaining life span.

        Returns:
            float: Opacity value between 0.0 and 1.0.
        """
        if self._static:
            return 1.0
        if self.isExpired():
            return 0.0
        return (self._expirationTime - time.time()) / self._expirationDuration

    @staticmethod
    def _invertTransform(transform: Transform) -> Transform:
        """
        Compute the inverse of a given transform.

        Args:
            transform (Transform): The transform to invert.

        Returns:
            Transform: The inverted transform.
        """
        matrix = tf.translation_matrix([transform.translation.x, transform.translation.y, transform.translation.z])
        rotation = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        matrix[:3, :3] = tf.quaternion_matrix(rotation)[:3, :3]
        inverted_matrix = tf.inverse_matrix(matrix)

        inverted_translation = tf.translation_from_matrix(inverted_matrix)
        inverted_rotation = tf.quaternion_from_matrix(inverted_matrix)

        inverted_transform = Transform()
        inverted_transform.translation.x, inverted_transform.translation.y, inverted_transform.translation.z = inverted_translation
        inverted_transform.rotation.x, inverted_transform.rotation.y, inverted_transform.rotation.z, inverted_transform.rotation.w = inverted_rotation

        return inverted_transform


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
        """
        Stop the background thread.
        """
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

            # Add forward and inverse transforms
            self._edges[header_frame_id][child_frame_id] = TransformRMV(header_frame_id, child_frame_id, transform, static, expiration)
            self._edges[child_frame_id][header_frame_id] = TransformRMV(child_frame_id, header_frame_id, TransformRMV._invertTransform(transform), static, expiration)

    def _removeExpiredEdges(self):
        """
        Remove all expired transforms from the graph.
        """
        for parent, children in list(self._edges.items()):
            for child, transform in list(children.items()):
                if transform.isExpired():
                    del self._edges[parent][child]
            if not self._edges[parent]:
                del self._edges[parent]

    def calculateTransform(self, start: str, end: str) :
        """
        Calculate the transform between two frames by traversing the graph.

        Args:
            start (str): The starting frame.
            end (str): The target frame.

        Returns:
            Transform: The resulting transform if a path exists, otherwise None.
        """
        path = self._getShortestPath(start, end)
        if not path:
            return None

        current_transform = None
        min_opacity = 1.0
        for i in range(len(path) - 1):
            parent, child = path[i], path[i + 1]
            transform = self._edges[parent][child].getTransform()
            min_opacity = min(min_opacity, self._edges[parent][child].getOpacity())
            if current_transform is None:
                current_transform = transform
            else:
                current_transform = self._combineTransforms(current_transform, transform)

        return current_transform, min_opacity

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

    @staticmethod
    def _combineTransforms(transform1: Transform, transform2: Transform) -> Transform:
        """
        Combine two transforms into a single transform.

        Args:
            transform1 (Transform): The first transform.
            transform2 (Transform): The second transform.

        Returns:
            Transform: The combined transform.
        """
        matrix1 = tf.translation_matrix([transform1.translation.x, transform1.translation.y, transform1.translation.z])
        rotation1 = [transform1.rotation.x, transform1.rotation.y, transform1.rotation.z, transform1.rotation.w]
        matrix1[:3, :3] = tf.quaternion_matrix(rotation1)[:3, :3]

        matrix2 = tf.translation_matrix([transform2.translation.x, transform2.translation.y, transform2.translation.z])
        rotation2 = [transform2.rotation.x, transform2.rotation.y, transform2.rotation.z, transform2.rotation.w]
        matrix2[:3, :3] = tf.quaternion_matrix(rotation2)[:3, :3]

        combined_matrix = tf.concatenate_matrices(matrix1, matrix2)

        combined_translation = tf.translation_from_matrix(combined_matrix)
        combined_rotation = tf.quaternion_from_matrix(combined_matrix)

        combined_transform = Transform()
        combined_transform.translation.x, combined_transform.translation.y, combined_transform.translation.z = combined_translation
        combined_transform.rotation.x, combined_transform.rotation.y, combined_transform.rotation.z, combined_transform.rotation.w = combined_rotation

        return combined_transform

    def getAllFrames(self) -> List[str]:
        """
        Retrieve all frames in the graph.

        Returns:
            List[str]: A sorted list of frame names.
        """
        with self._lock:
            return sorted(self._nodes)
