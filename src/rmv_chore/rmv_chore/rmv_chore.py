
from rclpy.node import Node

from library import dataManager, VisualizationParams, TimerLogger, SharedData
from visualization.visualization import Visualization
from std_msgs.msg import ColorRGBA
from threading import Thread, Lock
from time import sleep
class RmvChore:
    def __init__(self)->None:
        """
        Constructor for the RmvChore class.
        """
        self.node : Node= Node("rmv_chore", namespace="rmv")
        
        background_color = ColorRGBA(r=0.1, g=0.1, b=0.1, a=1.0)
        self.visu_params = VisualizationParams(width=800, height=600, fps=30, background_color=background_color)
        
        self.timer_logger : TimerLogger = TimerLogger(self.node, 2.0)
        self.timer_logger2 : TimerLogger = TimerLogger(self.node, 2.0)
        self.data_manager = dataManager(self.node)
        self.data_manager.start()
        self._is_running = True
        self.thread = Thread(target=self._updateMarkers, daemon=True)
        self.visualization = Visualization(self.node, self.visu_params)
        self.thread.start()
        
        print("Node rmv_chore created successfully")
        
    def __del__(self)->None: 
        """
        Destructor for the RmvChore class.
        """
        
    def _updateMarkers(self)->None:
        """
        Method to update the markers list and push filtered markers to the queue.
        """
        while self._is_running:
            self.timer_logger2.logExecutionTime(self.shareData)()
            self.timer_logger.logExecutionTime(self.visualization.visualize)()
            sleep(0.1)
        
    def shareData(self)->SharedData:
        """
        Method to get the shared data object.
        Returns:
            SharedData: The shared data object.
        """
        
        shared_data = self.data_manager.getSharedData()
        self.visualization.shared_data.update_main_tf(shared_data.get_main_tf())
        self.visualization.shared_data.update_other_tfs(shared_data.get_other_tfs())
        self.visualization.shared_data.update_markers(shared_data.get_markers())
        self.timer_logger.logExecutionTime(self.visualization.visualize)() 
        
        
   
        
    def getNode(self)->Node:
        """
        Method to get the node object.
        Returns:
            Node: The node object.
        """
        return self.node
    