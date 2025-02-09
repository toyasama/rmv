
from rclpy.node import Node

from library import dataManager, VisualizationParams, TimerLogger
from visualization.visualization import Visualization
from std_msgs.msg import ColorRGBA
from threading import Thread
from time import sleep
class RmvChore:
    def __init__(self)->None:
        """
        Constructor for the RmvChore class.
        """
        self.node : Node= Node("rmv_chore", namespace="rmv")
        
        background_color = ColorRGBA(r=0.1, g=0.1, b=0.1, a=1.0)
        self.visu_params = VisualizationParams(width=800, height=600, fps=30, background_color=background_color)
        
        self.visualization = Visualization(self.node, self.visu_params)
        self.timer_logger : TimerLogger = TimerLogger(self.node, 2.0)
        self.timer_logger2 : TimerLogger = TimerLogger(self.node, 2.0)
        self.timer_logger_loop : TimerLogger = TimerLogger(self.node, 2.0)
        self.data_manager = dataManager(self.node)
        self._is_running = True
        self.thread = Thread(target=self._updateMarkers, daemon=True)
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
            self.timer_logger_loop.logExecutionTime(self.loop)()
            sleep(0.1)
        
    def loop(self)->None:
        """
        Method to run the node.
        """
        self.timer_logger2.logExecutionTime(self.data_manager.run)()
        self.timer_logger.logExecutionTime(self.visualization.visualize)(self.data_manager.shared_data)
            
        
    def getNode(self)->Node:
        """
        Method to get the node object.
        Returns:
            Node: The node object.
        """
        return self.node
    