import time
from collections import defaultdict
from typing import Dict, Optional, List
from geometry_msgs.msg import Transform, TransformStamped
from threading import Thread, RLock
import tf_transformations as tf
import networkx as nx

from tf_management.transform import TransformRMV, TransformUtils

class FrameDrawingInfo:
    def __init__(self, frame: str, transform: Transform, start_connection: Transform, end_connection: Transform, opacity: float, valid: bool ):
        """Information for drawing a TF."""
        self.name = frame
        self.transform = transform
        self.opacity = opacity
        self.start_connection = start_connection
        self.end_connection = end_connection
        self.valid = valid

class Graph(Thread):
    def __init__(self):
        """
        Represents a graph of frames connected by transforms.
        """
        super().__init__()
        self._graph = nx.DiGraph()  # Directed graph from networkx
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

            # Add forward transform
            transform_rmv = TransformRMV(header_frame_id, child_frame_id, transform)
            transform_rmv.setStatic(static)
            transform_rmv.setExpirationDuration(expiration)
            transform_rmv.setInitialDirection(True)
            self._graph.add_edge(header_frame_id, child_frame_id, object=transform_rmv)

            # Add inverse transform
            inverse_transform = TransformUtils.invertTransform(transform)
            inverse_transform_rmv = TransformRMV(child_frame_id, header_frame_id, inverse_transform)
            inverse_transform_rmv.setStatic(static)
            inverse_transform_rmv.setExpirationDuration(expiration)
            inverse_transform_rmv.setInitialDirection(False)
            self._graph.add_edge(child_frame_id, header_frame_id, object=inverse_transform_rmv)

    def _removeExpiredEdges(self):
        """Remove all expired transforms from the graph."""
        expired_edges = []
        for u, v, data in self._graph.edges(data=True):
            if data['object'].isExpired():
                expired_edges.append((u, v))

        for u, v in expired_edges:
            self._graph.remove_edge(u, v)

    def calculateAllTransformsFrom(self, main_frame: str) -> List[FrameDrawingInfo]:
        """
        Calculate all relative transforms from a main frame to other frames.

        Args:
            main_frame (str): The main frame from which to calculate transforms.

        Returns:
            List[FrameDrawingInfo]: List of relative frame information.
        """
        with self._lock:
            if main_frame not in self._graph:
                return []

            transforms = []

            for target_frame in self._graph.nodes:
                if target_frame == main_frame:
                    continue

                try:
                    path = nx.shortest_path(self._graph, source=main_frame, target=target_frame)
                except nx.NetworkXNoPath:
                    continue

                current_transform = None
                min_opacity = 1.0
                start_connection = None
                end_connection = None
                combined_validity = True

                for i in range(len(path) - 1):
                    parent, child = path[i], path[i + 1]
                    edge_data = self._graph[parent][child]['object']
                    transform = edge_data.getTransform()
                    combined_validity = edge_data.isValid() and combined_validity

                    min_opacity = min(min_opacity, edge_data.getOpacity())

                    if current_transform is None:
                        current_transform = transform
                    else:
                        current_transform = TransformUtils.combineTransforms(current_transform, transform)

                if edge_data._initial_direction:
                    inverse_transform = self.getTransform(child, parent)
                    start_connection = TransformUtils.combineTransforms(current_transform, inverse_transform)
                    end_connection = current_transform
                else:
                    inverse_transform = self.getTransform(child, parent)
                    end_connection = TransformUtils.combineTransforms(current_transform, inverse_transform)
                    start_connection = current_transform
                    
                transforms.append(FrameDrawingInfo(
                    frame=target_frame,
                    transform=current_transform,
                    start_connection=start_connection,
                    end_connection=end_connection,
                    opacity=min_opacity,
                    valid=combined_validity
                ))

            return transforms

    def getAllFrames(self) -> List[str]:
        """Retrieve all frames in the graph."""
        with self._lock:
            return list(self._graph.nodes)

    def getTransform(self, parent: str, child: str) -> Optional[Transform]:
        """Retrieve the transform between two frames."""
        with self._lock:
            edge_data = self._graph.get_edge_data(parent, child, default=None)
            if edge_data:
                return edge_data['object'].getTransform()
            return None
