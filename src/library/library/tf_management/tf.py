from typing import List
from rclpy.node import Node
from rclpy.time import Duration
from tf2_msgs.msg import TFMessage
from .graph import TransformGraph, FrameRMV
from geometry_msgs.msg import  Transform
from ..utils.timer_log import TimerLogger
from threading import Thread, Lock
from time import sleep

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
        
        self._lock_main_frame = Lock()
        self._lock_all_frame = Lock()
        
        self.node.create_subscription(TFMessage, '/tf', self.tfCallback, 10)
        self.node.create_subscription(TFMessage, '/tf_static', self.tfStaticCallback, 10)

        self.node.get_logger().info("TFManager initialized successfully.")
        self._thread = Thread(target=self._updateAllTransformsFrom, daemon=True).start()

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
            
    def _updateAllTransformsFrom(self) -> None:
        """
        Met à jour les transformations à partir du frame principal.
        """
        while True:
            with self._lock_main_frame:
                all_info: List[FrameRMV] = self.evaluateTransformsFrom(self.main_frame_name)
            with self._lock_all_frame:
                self.all_transform_from_main_frame =  {frame.name: frame for frame in all_info}
            self.setDefaultMainFrame()
            sleep(0.1)
            

    def setDefaultMainFrame(self)->None:
        """
        Set the main TF frame to the first available frame if ones.
        """
        frames =  self.frames
        with self._lock_main_frame:
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
        with self._lock_main_frame:
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
        with self._lock_main_frame:
            return frame == self.main_frame_name
    
    def getAllTransformsFromMainFrame(self)->dict[str,FrameRMV]:
        """
        Get all the transforms from the main frame.
        Returns:
            dict[str,FrameRMV]: The dictionary of transforms from the main frame.
        """
        with self._lock_all_frame:
            return self.all_transform_from_main_frame

   