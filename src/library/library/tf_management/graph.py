import time
import networkx as nx
from threading import Thread, RLock
from typing import Optional, List
from geometry_msgs.msg import Transform, TransformStamped
from .transform_utils import TransformUtils
from .transform_rmv import RmvTransform, TransformDrawerInfo
from abc import ABC, abstractmethod
from typing import Dict
class BaseGraph(ABC):
    """Base class managing a directed graph with thread-safe operations."""
    def __init__(self):
        self._graph = nx.DiGraph()
        self._graph_lock = RLock()
        self._running = True
        self.main_frame: Optional[str] = "frame_1"
        self._lock_main_frame = RLock()
        self.count = 0
        self.begin = time.time()
        
        self._worker_thread = Thread(target=self._workerLoop, daemon=True)
        self._worker_thread.start()

    def __del__(self):
        self._running = False
        self._worker_thread.join()
        
    def _workerLoop(self):
        """Continuous worker loop handling updates and removals."""
        while self._running:
            self._update()
            self._removeExpiredEdges()
            self.setDefaultMainFrame()
            time.sleep(0.1)

    @abstractmethod
    def _update(self):
        """Update the graph and compute transforms."""
        pass
    
    def _removeExpiredEdges(self):
        """Removes all expired edges from the graph."""
        to_remove = []
        with self._graph_lock:
            for u, v, data in self._graph.edges(data=True):
                if data["frameInfo"].isExpired:
                    to_remove.append((u, v))
            
            for u, v in to_remove:
                self._graph.remove_edge(u, v)

    @abstractmethod
    def setDefaultMainFrame(self):
        pass
            
class TransformGraph(BaseGraph):
    
    def __init__(self):
        super().__init__()
    
    """Graph that manages frame transforms."""
    def addTransform(self, transform_stamped: TransformStamped, static: bool = False):
        """Add a transform to the graph from a TransformStamped message."""
        parent_frame = transform_stamped.header.frame_id
        child_frame = transform_stamped.child_frame_id
        with self._graph_lock:
            if (parent_frame, child_frame) in self._graph.edges:
                self._graph[parent_frame][child_frame]["frameInfo"].update(transform_stamped)
                self._graph[child_frame][parent_frame]["frameInfo"].update(self.inverseTransformMsg(transform_stamped))
                return
                

        forward_transform = RmvTransform(transform_stamped, static)
        inverse_transform = RmvTransform(self.inverseTransformMsg(transform_stamped), static)

        with self._graph_lock:
            self._graph.add_edge(parent_frame, child_frame, frameInfo=forward_transform)
            self._graph.add_edge(child_frame, parent_frame, frameInfo=inverse_transform)
        
    def inverseTransformMsg(self, transform_stamped: TransformStamped) -> TransformStamped:
        """Compute the inverse of a TransformStamped."""
        inverse_transform_stamped = TransformStamped()
        inverse_transform = TransformUtils.invertTransform(transform_stamped.transform)
        inverse_transform_stamped.header.frame_id = transform_stamped.child_frame_id
        inverse_transform_stamped.child_frame_id = transform_stamped.header.frame_id
        inverse_transform_stamped.header.stamp = transform_stamped.header.stamp
        inverse_transform_stamped.transform = inverse_transform
        return inverse_transform_stamped

    def getTransform(self, parent: str, child: str) -> Optional[Transform]:
        """Retrieve the transform between two frames."""
        with self._graph_lock:
            edge_data = self._graph.get_edge_data(parent, child)
            return edge_data["frameInfo"].transform if edge_data else None
    
    def _update(self):
        """Periodically update transforms and evaluate transforms from the main frame."""
        if self.main_frame not in self._graph:
            return
        with self._graph_lock:
            for parent, child in self._graph.edges:
                edge_data: RmvTransform = self._graph[parent][child]["frameInfo"]
                if edge_data.drawer_info.main_frame == self.main_frame or edge_data.name == self.main_frame:
                    continue
                try:
                    path = nx.shortest_path(self._graph, source=self.main_frame, target=edge_data.name)
                    transform_from_main = self._computeTransformInfo(path)
                    if not transform_from_main:
                        continue
                    if edge_data._initial_direction:
                        inverse_transform = self.getTransform(child, parent)
                        start_connection = TransformUtils.combineTransforms(transform_from_main, inverse_transform)
                        end_connection = transform_from_main
                        self._graph[parent][child]["frameInfo"].updateTransformDrawerInfo(self.main_frame,transform_from_main, start_connection, end_connection)
                    else:
                        inverse_transform = self.getTransform(child, parent)
                        end_connection = TransformUtils.combineTransforms(transform_from_main, inverse_transform)
                        start_connection = transform_from_main
                        self._graph[child][parent]["frameInfo"].updateTransformDrawerInfo(self.main_frame,transform_from_main, start_connection, end_connection)
                except nx.NetworkXNoPath:
                    continue

    def _computeTransformInfo(self, path: List[str]):
        """Compute the combined transform for a given path."""
        current_transform = None
        edge_data = None
        for i in range(len(path) - 1):
            parent, child = path[i], path[i + 1]
            edge_data: RmvTransform = self._graph[parent][child]["frameInfo"]
            transform = edge_data.transform
            current_transform = TransformUtils.combineTransforms(current_transform, transform) if current_transform else transform
        return current_transform
        

    @property
    def frames(self) -> List[str]:
        """Retrieve all frames in the graph."""
        with self._graph_lock:
            return list(self._graph.nodes)

    def setDefaultMainFrame(self):
        """
        Set the main TF frame to the first available frame if ones.
        """
        frames =  self.frames
        if not frames:
            return
        with self._lock_main_frame:
            if frames and frames[self.count] != self.main_frame:
                self.main_frame = frames[self.count]
                print(f"Main frame set to: {self.main_frame}")
        elapsed = time.time() - self.begin
        if elapsed > 30.0 and frames:
            self.begin = time.time()
            self.count = (self.count +1) % len(frames)
            
    def getTransformsFromMainFrame(self):
        """
        Get all transforms from the main frame.
        """
        drawer_list = []
        with self._graph_lock:
            for parent, child in self._graph.edges:
                edge_data: RmvTransform = self._graph[parent][child]["frameInfo"]
                if edge_data.drawer_info.main_frame == self.main_frame:
                    drawer_list.append(edge_data.drawer_info)
            return list(drawer_list)