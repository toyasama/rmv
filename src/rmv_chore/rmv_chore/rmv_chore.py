
from rclpy.node import Node
from topic_management.topic_manager import TopicManager
from markers_management.markers_manager import MarkersManager
from markers_management.markers import MarkerRmv
from utils.timer_log import TimerLogger
from tf_management.tf import TFManager
from pprint import pprint

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
        self.tf_manager = TFManager(self.node, Params.TIMEOUT_TF_BUFFER)
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
        self.tf_manager.setDefaultMainFrame() # Only for testing purposes
        filtered_markers = self._filterMarkersInMainTfFrame(markers_rmv)
        
        # if len(self.markers_manager.getMarkersList()) > 0:
        #     if self.tf_manager.canTransform("test", "test2"):
        #         self.tf_manager.transformPoint(self.markers_manager.getMarkersList()[0], "test", "test2")
                
        pprint(self.tf_manager.getAvailableFrames())
        print("info")
        pprint([marker.getIdentifier() for marker in self.markers_manager.getMarkersList()])
        pprint([marker.getIdentifier() for marker in filtered_markers])
        
    def _filterMarkersInMainTfFrame(self, markers_rmv:list[MarkerRmv])->list[MarkerRmv]:
        """
        Update the markers list to the main TF frame.
        Args:
            markers_rmv (list[MarkerRmv]): The list of markers to be updated.
        """
        filtered_markers :list[MarkerRmv] = []
        for marker in markers_rmv:
            if self.tf_manager.canTransform(marker.getTfFrame(), self.tf_manager.main_tf_frame):
                transform_point = self.tf_manager.transformPose(marker, marker.getTfFrame(), self.tf_manager.main_tf_frame)
                if transform_point:
                    marker.setFrameId(self.tf_manager.main_tf_frame)
                    marker.setPose(transform_point)
                    filtered_markers.append(marker)
                    
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
