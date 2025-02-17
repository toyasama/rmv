from .markers import MarkerRmv
from typing import List
from rclpy.node import Node
from ..topic_management.topic_manager import TopicManager
from ..tf_management.transform_utils import TransformUtils
from ..utils.shared_data import SharedData
from ..utils.timer_log import TimerLogger
from ..tf_management.tf import TFManager
    

class dataManager():
    def __init__(self, node: Node, shared_data:SharedData)-> None:
        """
        Constructor for the DataManager class.
        Args:
            node (Node): The ROS2 node.
        """
        super().__init__()
        self._shared_data = shared_data
        self.markers_manager: TopicManager= TopicManager(node, 0.1)
       
        self.timer_logger_tf = TimerLogger(node, 2.0)
    
    def processData(self):
        """
        Run the data manager thread.
        """
        # self.timer_logger_tf.logExecutionTime(self._process)()
        self._process()
    

    
    @property
    def shared_data(self)->SharedData:
        """
        Get the shared data object.

        Returns:
            SharedData: The shared data object.
        """
        return self._shared_data
    
