import time
import networkx as nx
from typing import  Optional, List
from threading import Thread, RLock
from geometry_msgs.msg import Transform, TransformStamped
from .frame_rmv import FrameRMV
from .transform_utils import  TransformUtils
from .transform_rmv import TransformRMV

class BaseGraph:
    """Base class managing a directed graph with thread-safe operations."""
    def __init__(self):
        self._graph = nx.DiGraph()
        self._graph_lock = RLock()
        self._running = True
        self._thread = Thread(target=self._removePeriodically, daemon=True)
        self._thread.start()

    def __del__(self):
        self._running = False
        self._thread.join()

    def _removePeriodically(self):
        """Periodically removes expired edges."""
        while self._running:
            self._removeExpiredEdges()
            time.sleep(1)

    def _removeExpiredEdges(self):
        """Removes all expired edges from the graph."""
        with self._graph_lock:
            expired_edges = [(u, v) for u, v, data in self._graph.edges(data=True) if data["frameInfo"].isExpired]
            for u, v in expired_edges:
                self._graph.remove_edge(u, v)

class TransformGraph(BaseGraph):
    """Graph that manages frame transforms."""
    
    def addTransform(self, transform_stamped: TransformStamped, static: bool = False, expiration: float = 0):
        """
        Add a transform to the graph from a TransformStamped message.
        """
        parent_frame = transform_stamped.header.frame_id
        child_frame = transform_stamped.child_frame_id
        transform = transform_stamped.transform

        forward_transform  = TransformRMV(parent_frame, child_frame, transform)
        forward_transform.isStatic = static
        forward_transform.expiration_duration =expiration

        inverse_transform = TransformUtils.invertTransform(transform)
        inverse_transform_rmv = TransformRMV(child_frame, parent_frame, inverse_transform)
        inverse_transform_rmv.isStatic = static
        inverse_transform_rmv.expiration_duration =expiration

        with self._graph_lock:
            self._graph.add_edge(parent_frame, child_frame, frameInfo=forward_transform)
            self._graph.add_edge(child_frame, parent_frame, frameInfo=inverse_transform_rmv)

    def getTransform(self, parent: str, child: str) -> Optional[Transform]:
        """Retrieve the transform between two frames."""
        with self._graph_lock:
            edge_data = self._graph.get_edge_data(parent, child)
            return edge_data["frameInfo"].getTransform() if edge_data else None

    def evaluateTransformsFrom(self, main_frame: str) -> List[FrameRMV]:
        """Evaluate all relative transforms from a main frame."""
        with self._graph_lock:
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

                transform_info = self._computeTransformInfo(path)
                transform_info.frame = target_frame
                transforms.append(transform_info)

            return transforms

    def _computeTransformInfo(self, path: List[str]) -> FrameRMV:
        """Compute the combined transform for a given path."""
        current_transform = None
        min_opacity = 1.0
        combined_validity = True

        for i in range(len(path) - 1):
            parent, child = path[i], path[i + 1]
            edge_data:TransformRMV = self._graph[parent][child]["frameInfo"]

            combined_validity &= edge_data.isValid
            min_opacity = min(min_opacity, edge_data.opacity)

            transform = edge_data.getTransform()
            current_transform = TransformUtils.combineTransforms(current_transform, transform) if current_transform else transform

        if edge_data._initial_direction:
            inverse_transform = self.getTransform(child, parent)
            start_connection = TransformUtils.combineTransforms(current_transform, inverse_transform)
            end_connection = current_transform
        else:
            inverse_transform = self.getTransform(child, parent)
            end_connection = TransformUtils.combineTransforms(current_transform, inverse_transform)
            start_connection = current_transform
        return FrameRMV().fill(
            frame=path[-1],
            transform=current_transform,
            start_connection=start_connection,
            end_connection=end_connection,
            opacity=min_opacity,
            valid=combined_validity
        )

    @property
    def frames(self) -> List[str]:
        """Retrieve all frames in the graph."""
        with self._graph_lock:
            return list(self._graph.nodes)
