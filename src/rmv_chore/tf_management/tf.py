from typing import List, Tuple, Generator
from rclpy.node import Node
from rclpy.time import Duration
from tf2_msgs.msg import TFMessage
from tf_management.graph import Graph, FrameDrawingInfo, TransformUtils
from geometry_msgs.msg import  Transform, Pose
from utils.timer_log import TimerLogger
import tf_transformations as tf

class TFManager:
    def __init__(self, node: Node, buffer_timeout: float, cache_duration: float = 3.0) -> None:
        """
        Gestionnaire pour la transformation TF avec structures avancées.
        """
        self.node = node
        self.timer_logger = TimerLogger(self.node, 5.0)


        self.main_frame_name = ""
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
            self.graph.addEdgeFromTransformStamped(transform,expiration= self.expiration_duration, static=False)

    def tfStaticCallback(self, msg: TFMessage) -> None:
        """
        Callback pour les transformations statiques.
        """
        for transform in msg.transforms:
            self.graph.addEdgeFromTransformStamped(transform, static=True)


    def getAvailableTFNames(self) -> List[str]:
        """
        Récupère la liste des noms des TF disponibles à partir des buffers statiques et des relations parent-enfant.
        
        Returns:
            List[str]: Liste des noms des frames disponibles.
        """
        available_frames = self.graph.getAllFrames()
        # if  available_frames: print(available_frames)
        return available_frames


    def setDefaultMainFrame(self)->None:
        
        """
        Set the main TF frame to the first available frame if ones.
        """
        frames = self.getAvailableTFNames()
        if frames:
            self.main_frame_name = frames[self.count]
            
        if self.node.get_clock().now() - self.start_time > Duration(seconds=5) and frames:
            self.start_time = self.node.get_clock().now()
            self.count = (self.count +1) % len(frames)
        
    def getMainFrame(self)->str:
        """
        Get the main TF frame for the TF manager.
        Returns:
            str: The name of the main TF frame.
        """
        return self.main_frame_name
    
    def equalMainFrame(self, frame: str)->bool:
        """
        Check if the frame is the main frame.
        Args:
            frame (str): The frame to check.
        Returns:
            bool: True if the frame is the main frame, False otherwise.
        """
        return frame == self.main_frame_name
    
    def getRelativeTransforms(self) -> List[FrameDrawingInfo]:
        """
        Get the relative transforms of the main frame with respect to the other frames.

        Returns:
            List[FrameDrawingInfo]: List of relative frame information.
        """
        return self.timer_logger.logExecutionTime(self._getRelativeTransforms)()
    

    def _getRelativeTransforms(self) -> List[FrameDrawingInfo]:
        """
        Optimized method to get all relative transforms from the main frame.

        Returns:
            List[FrameDrawingInfo]: List of relative frame information.
        """
        if not self.main_frame_name:
            return []

        return self.graph.calculateAllTransformsFrom(self.main_frame_name)
    

   