
from rclpy.node import Node
from topic_management.topic_manager import TopicManager
from markers_management.markers_manager import MarkersManager
from markers_management.markers import MarkerRmv
from utils.timer_log import TimerLogger
from tf_management.tf import TFManager
from pprint import pprint
import math
from rclpy.time import Duration

class  Params():
    """
    RMV Chore parameters

    Args:
        UPDATE_TOPIC_PROCESS_PERIOD (float): Period to check for markers topics in topic manager.
        UPDATE_MARKERS_PROCESS_PERIOD (float)
    """
    UPDATE_TOPIC_PROCESS_PERIOD = 0.25
    UPDATE_MARKERS_PROCESS_PERIOD = 0.1
    TIMEOUT_TF_BUFFER = 0.05
    CACHE_TIME = 3.0

class RmvChore:
    def __init__(self)->None:
        """
        Constructor for the RmvChore class.
        """
        self.node : Node= Node("rmv_chore")
        
        self.topic_manager : TopicManager = TopicManager(self.node, Params.UPDATE_TOPIC_PROCESS_PERIOD)
        self.timer_test = self.node.create_timer(Params.UPDATE_MARKERS_PROCESS_PERIOD, self._updateMarkersCallBack)
        self.markers_manager = MarkersManager(self.node)
        self.timer_logger = TimerLogger(self.node, 5.0)
        
        seconds = math.floor(Params.CACHE_TIME)
        nanoseconds = int((Params.CACHE_TIME - seconds) * 1e9)
        self.cache_time = Duration(seconds=seconds, nanoseconds=nanoseconds)
        self.tf_manager = TFManager(self.node, Params.TIMEOUT_TF_BUFFER, self.cache_time)
        
        print("Node rmv_chore created successfully")
        
    def _updateMarkersCallBack(self)->None:
        """
        Timer callback method.
        """
        self.timer_logger.logExecutionTime(self._updateMarkers)()
        
    def _updateMarkers(self)->None:
        """
        Method to update the markers list.
        """
        markers = self.topic_manager.getMarkersList()
        self.topic_manager.cleanMarkersList()
        self.markers_manager.processNewMarkers(markers)
        
        markers_rmv = self.markers_manager.getMarkersList()
        if not self.tf_manager.getMainTfFrame():    # Only for testing purposes
            self.tf_manager.setDefaultMainFrame() 
        filtered_markers = self._filterMarkersInMainTfFrame(markers_rmv)
        
                
        # pprint(self.tf_manager.getAvailableFrames())
        # print("info")
        # pprint([marker.getIdentifier() for marker in self.markers_manager.getMarkersList()])
        # pprint([marker.getIdentifier() for marker in filtered_markers])
        
    def _filterMarkersInMainTfFrame(self, markers_rmv:list[MarkerRmv])->list[MarkerRmv]:
        """
        Update the markers list to the main TF frame.
        Args:
            markers_rmv (list[MarkerRmv]): The list of markers to be updated.
        """
        filtered_markers :list[MarkerRmv] = []
        for marker in markers_rmv:
            
            if self.tf_manager.isInMainTfFrame(marker.getTfFrame()):
                filtered_markers.append(marker)
                continue
                
            if self.tf_manager.canTransform(marker.getTfFrame(), self.tf_manager.main_tf_frame):
                transform_point = self.tf_manager.transformPose(marker.getPose(), marker.getTfFrame(), self.tf_manager.main_tf_frame)
                if transform_point:
                    marker.setFrameId(self.tf_manager.main_tf_frame)
                    marker.setPose(transform_point)
                    filtered_markers.append(marker)
            else:
                print(f"Cannot transform {marker.getIdentifier()} from {marker.getTfFrame()} to {self.tf_manager.main_tf_frame}")
            
                    
        return filtered_markers
        
    def getNode(self)->Node:
        """
        Method to get the node object.
        Returns:
            Node: The node object.
        """
        return self.node
    
    def destroyNode(self)->None:
        """
        Method to destroy the node object.
        """
        self.node.destroy_node()
        print("Node rmv_chore destroyed")
