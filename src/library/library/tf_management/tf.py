from typing import List
from rclpy.node import Node
from rclpy.time import Duration
from tf2_msgs.msg import TFMessage
from .graph import TransformGraph, FrameRMV
from geometry_msgs.msg import  Transform
from ..utils.timer_log import TimerLogger

class TFManager(TransformGraph):
    def __init__(self, node: Node) -> None:
        """
        Gestionnaire pour la transformation TF avec structures avancées.
        """
        super().__init__()
        self.node = node
        self.timer_logger = TimerLogger(self.node, 2.0)

        self.main_frame_name = ""
        self.all_transform_from_main_frame : dict[str,FrameRMV] =  {}
        self.start_time = self.node.get_clock().now()
        self.frame_index = 0
        self.count = 0
        self.expiration_duration = 5.0
        
        self.node.create_subscription(TFMessage, '/tf', self.tfCallback, 10)
        self.node.create_subscription(TFMessage, '/tf_static', self.tfStaticCallback, 10)

        self.node.get_logger().info("TFManager initialized successfully.")

    def tfCallback(self, msg: TFMessage) -> None:
        """
        Callback pour les transformations dynamiques.
        """
        for transform in msg.transforms:
            self.addTransform(transform,expiration= self.expiration_duration, static=False)
            # self.timer_logger.logExecutionTime(self.addTransform)(transform,expiration= self.expiration_duration, static=False)

    def tfStaticCallback(self, msg: TFMessage) -> None:
        """
        Callback pour les transformations statiques.
        """
        for transform in msg.transforms:
            self.addTransform(transform, static=True)
            # self.timer_logger.logExecutionTime(self.addTransform)(transform, static=True)
            
    def updateAllTransformsFrom(self) -> None:
        """
        Met à jour les transformations à partir du frame principal.
        """
        all_info: List[FrameRMV] = self.evaluateTransformsFrom(self.main_frame_name)
        self.all_transform_from_main_frame =  {frame.name: frame for frame in all_info}
            
    def getAvailableTFNames(self) -> List[str]:
        """
        Récupère la liste des noms des TF disponibles à partir des buffers statiques et des relations parent-enfant.
        
        Returns:
            List[str]: Liste des noms des frames disponibles.
        """
        available_frames = self.frames
        return available_frames

    def setDefaultMainFrame(self)->None:
        """
        Set the main TF frame to the first available frame if ones.
        """
        frames = self.getAvailableTFNames()
        if frames and frames[self.count] != self.main_frame_name:
            self.main_frame_name = frames[0]
                
        if self.node.get_clock().now() - self.start_time > Duration(seconds=20) and frames:
            self.start_time = self.node.get_clock().now()
            self.count = (self.count +1) % len(frames)
            
    def getMainFrame(self)->FrameRMV:
        """
        Get the main TF frame for the TF manager.
        Returns:
            str: The name of the main TF frame.
        """
        return FrameRMV().fill(
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
        return frame == self.main_frame_name
    
    def getAllTransformsFromMainFrame(self)->dict[str,FrameRMV]:
        """
        Get all the transforms from the main frame.
        Returns:
            dict[str,FrameRMV]: The dictionary of transforms from the main frame.
        """
        return self.all_transform_from_main_frame

   