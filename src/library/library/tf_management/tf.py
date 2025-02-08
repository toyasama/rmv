from typing import List, Tuple, Generator
from rclpy.node import Node
from rclpy.time import Duration
from tf2_msgs.msg import TFMessage
from .graph import Graph, FrameDrawingInfo, TransformUtils
from geometry_msgs.msg import  Transform, Pose
from ..utils.timer_log import TimerLogger
import tf_transformations as tf
from threading import Thread, RLock
from time import sleep

class TFManager(Thread):
    def __init__(self, node: Node, buffer_timeout: float, cache_duration: float = 3.0) -> None:
        """
        Gestionnaire pour la transformation TF avec structures avancées.
        """
        super().__init__()
        self.node = node
        self.timer_logger = TimerLogger(self.node, 5.0)

        self._lock_graph = RLock()
        self._lock_main_frame = RLock()
        self.__running = True

        self.main_frame_name = ""
        self.all_transform_from_main_frame : dict[str,FrameDrawingInfo] =  {}
        self.start_time = self.node.get_clock().now()
        self.frame_index = 0
        self.count = 0
        self.expiration_duration = 5.0
        
        self.graph = Graph()

        self.node.create_subscription(TFMessage, '/tf', self.tfCallback, 10)
        self.node.create_subscription(TFMessage, '/tf_static', self.tfStaticCallback, 10)
        self.graph.start()
        self.node.get_logger().info("TFManager initialized successfully.")

    def tfCallback(self, msg: TFMessage) -> None:
        """
        Callback pour les transformations dynamiques.
        """
        for transform in msg.transforms:
            with self._lock_graph:
                self.graph.addEdgeFromTransformStamped(transform,expiration= self.expiration_duration, static=False)

    def tfStaticCallback(self, msg: TFMessage) -> None:
        """
        Callback pour les transformations statiques.
        """
        for transform in msg.transforms:
            with self._lock_graph:
                self.graph.addEdgeFromTransformStamped(transform, static=True)

    def run(self):
        while self.__running:
            self.updateAllTransformsFrom()
            self.new_main_frame = False
            sleep(0.1)
            
    def updateAllTransformsFrom(self) -> None:
        """
        Met à jour les transformations à partir du frame principal.
        """
        with self._lock_main_frame and self._lock_graph:
            all_info: List[FrameDrawingInfo] = self.graph.calculateAllTransformsFrom(self.main_frame_name)
            self.all_transform_from_main_frame =  {frame.name: frame for frame in all_info}
            
    def getAvailableTFNames(self) -> List[str]:
        """
        Récupère la liste des noms des TF disponibles à partir des buffers statiques et des relations parent-enfant.
        
        Returns:
            List[str]: Liste des noms des frames disponibles.
        """
        with self._lock_graph:
            available_frames = self.graph.getAllFrames()
        return available_frames

    def setDefaultMainFrame(self)->None:
        """
        Set the main TF frame to the first available frame if ones.
        """
        frames = self.getAvailableTFNames()
        with self._lock_main_frame:
            if frames and frames[self.count] != self.main_frame_name:
                self.main_frame_name = frames[0]
                
        if self.node.get_clock().now() - self.start_time > Duration(seconds=20) and frames:
            self.start_time = self.node.get_clock().now()
            self.count = (self.count +1) % len(frames)
            
    def getMainFrame(self)->FrameDrawingInfo:
        """
        Get the main TF frame for the TF manager.
        Returns:
            str: The name of the main TF frame.
        """
        with self._lock_main_frame:
            return FrameDrawingInfo().fill(
                    frame=self.main_frame_name,
                    transform=Transform(),
                    start_connection=None,
                    end_connection=None,
                    opacity=1.0,
                    valid=1
                )
    
    def equalMainFrame(self, frame: str)->bool:
        """
        Check if the frame is the main frame.
        Args:
            frame (str): The frame to check.
        Returns:
            bool: True if the frame is the main frame, False otherwise.
        """
        with self._lock_main_frame:
            return frame == self.main_frame_name
    
    def getAllTransformsFromMainFrame(self)->dict[str,FrameDrawingInfo]:
        """
        Get all the transforms from the main frame.
        Returns:
            dict[str,FrameDrawingInfo]: The dictionary of transforms from the main frame.
        """
        with self._lock_main_frame:
            return self.all_transform_from_main_frame

   