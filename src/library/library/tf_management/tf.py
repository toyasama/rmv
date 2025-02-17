from typing import List
from rclpy.node import Node
from rclpy.time import Duration
from tf2_msgs.msg import TFMessage
from ..utils.timer_log import TimerLogger
from threading import Thread, Lock
from time import sleep
from ..tf_management.graph import TransformGraph

class TFManager():
    def __init__(self, node: Node, transform_graph: TransformGraph) -> None:
        """
        Gestionnaire pour la transformation TF avec structures avancées.
        """
        super().__init__()
        self.node = node
        self.timer_logger = TimerLogger(self.node, 2.0)

        self.main_frame_name = ""
        self.all_transform_from_main_frame   =  {}
        self.frame_index = 0
        self.count = 0
        self.transform_graph = transform_graph
        
        self._lock_main_frame = Lock()
        self._lock_all_frame = Lock()
        
        self.node.create_subscription(TFMessage, '/tf', self.tfCallback, 10)
        self.node.create_subscription(TFMessage, '/tf_static', self.tfStaticCallback, 10)

        self.node.get_logger().info("TFManager initialized successfully.")

    def tfCallback(self, msg: TFMessage) -> None:
        """
        Callback pour les transformations dynamiques.
        """
        for transform in msg.transforms:
            self.transform_graph.addTransform(transform, static=False)
            # self.timer_logger.logExecutionTime(self.addTransform)(transform,expiration= self.expiration_duration, static=False)

    def tfStaticCallback(self, msg: TFMessage) -> None:
        """
        Callback pour les transformations statiques.
        """
        for transform in msg.transforms:
            self.transform_graph.addTransform(transform, static=True)
            # self.timer_logger.logExecutionTime(self.addTransform)(transform, static=True)
            

    # def getMainFrame(self):
    #     """
    #     Get the main TF frame for the TF manager.
    #     Returns:
    #         str: The name of the main TF frame.
    #     """
    #     with self._lock_main_frame:
    #         return FrameRMV().fill(
    #                 frame=self.main_frame_name,
    #                 transform=Transform(),
    #                 start_connection=None,
    #                 end_connection=None,
    #                 opacity=1.0,
    #                 valid=1
    #             )
    
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
    
    # def getAllTransformsFromMainFrame(self)->dict[str,FrameRMV]:
    #     """
    #     Get all the transforms from the main frame.
    #     Returns:
    #         dict[str,FrameRMV]: The dictionary of transforms from the main frame.
    #     """
    #     with self._lock_all_frame:
    #         return self.all_transform_from_main_frame

   