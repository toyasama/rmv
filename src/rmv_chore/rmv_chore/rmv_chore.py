
from rclpy.node import Node
from markers_management.markers_manager import MarkersManager,dataManager
from markers_management.markers import MarkerRmv
from utils.timer_log import TimerLogger
from pprint import pprint
import math
from visualization.visualization import Visualization
from std_msgs.msg import ColorRGBA
from typing import Tuple, List
from tf_management.tf import  FrameDrawingInfo, TransformUtils
from rmv_chore.shared_data import SharedData
from parameters.params import RmvParams , VisualizationParams
import copy
class RmvChore:
    def __init__(self)->None:
        """
        Constructor for the RmvChore class.
        """
        self.node : Node= Node("rmv_chore", namespace="rmv")
        
        self.shared_data: SharedData = SharedData()
        background_color = ColorRGBA(r=0.1, g=0.1, b=0.1, a=1.0)
        visu_params = VisualizationParams(width=800, height=600, fps=30, background_color=background_color)
        
        self.timer_test = self.node.create_timer(1/visu_params.fps, self._updateMarkersCallBack)
        
        self.timer_logger = TimerLogger(self.node, 5.0)
        self.data_manager = dataManager(self.node)
        self.data_manager.start()
        
        
        
        self.visualization = Visualization(self.node, visu_params, self.shared_data)
        
        print("Node rmv_chore created successfully")
        
    def __del__(self)->None: 
        """
        Destructor for the RmvChore class.
        """
        
    def _updateMarkers(self)->None:
        """
        Method to update the markers list and push filtered markers to the queue.
        """
        
        shared_data = self.data_manager.getSharedData()
        self.shared_data.update_main_tf(shared_data.get_main_tf())
        self.shared_data.update_other_tfs(shared_data.get_other_tfs())
        self.shared_data.update_markers(shared_data.get_markers())
        
        self.timer_logger.logExecutionTime(self.visualization.run)()
        
    def _updateMarkersCallBack(self)->None:
        """
        Timer callback method.
        """
        self._updateMarkers()
        
   

        
    def getNode(self)->Node:
        """
        Method to get the node object.
        Returns:
            Node: The node object.
        """
        return self.node
    