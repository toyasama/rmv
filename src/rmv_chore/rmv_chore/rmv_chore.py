
from rclpy.node import Node
from topic_management.topic_manager import TopicManager
from markers_management.markers_manager import MarkersManager
from markers_management.markers import MarkerRmv
from utils.timer_log import TimerLogger
from tf_management.tf import TFManager
from pprint import pprint
import math
from visualization.visualization import Visualization
from std_msgs.msg import ColorRGBA
from typing import Tuple, List
from tf_management.tf import  FrameDrawingInfo, TransformUtils
from rmv_chore.shared_data import SharedData
from parameters.params import RmvParams , VisualizationParams

class RmvChore:
    def __init__(self)->None:
        """
        Constructor for the RmvChore class.
        """
        self.node : Node= Node("rmv_chore", namespace="rmv")
        
        self.shared_data: SharedData = SharedData()
        rmv_params: RmvParams = RmvParams()
        
        self.topic_manager : TopicManager = TopicManager(self.node, rmv_params.UPDATE_TOPIC_PROCESS_PERIOD)
        self.timer_test = self.node.create_timer(rmv_params.UPDATE_MARKERS_PROCESS_PERIOD, self._updateMarkersCallBack)
        
        self.markers_manager = MarkersManager(self.node)
        self.timer_logger = TimerLogger(self.node, 5.0)
        
        self.tf_manager = TFManager(self.node, rmv_params.TIMEOUT_TF_BUFFER)
        
        background_color = ColorRGBA(r=0.1, g=0.1, b=0.1, a=1.0)
        rmv_params = VisualizationParams(width=800, height=600, fps=30, background_color=background_color)
        
        self.visualization = Visualization(self.node, rmv_params, self.shared_data)
        self.visualization.start()
        
        print("Node rmv_chore created successfully")
        
    def __del__(self)->None: 
        """
        Destructor for the RmvChore class.
        """
        self.visualization.stop()
        self.visualization.join()
        
    def _updateMarkers(self)->None:
        """
        Method to update the markers list and push filtered markers to the queue.
        """
        markers = self.topic_manager.extractMarkersList()
        self.topic_manager.cleanMarkersList()
        self.markers_manager.processNewMarkers(markers)
        
        markers_rmv = self.markers_manager.getMarkersList()
        self.tf_manager.setDefaultMainFrame() 
        self.shared_data.update_main_tf(self.tf_manager.getMainFrame())
        
        frames:FrameDrawingInfo = self.tf_manager.getRelativeTransforms()
        self.shared_data.update_other_tfs(frames) 
        
        filtered_markers = self._filterMarkersInMainTfFrame(markers_rmv, frames)
        self.shared_data.update_markers(filtered_markers)
        
        
        
        
    def _updateMarkersCallBack(self)->None:
        """
        Timer callback method.
        """
        self.timer_logger.logExecutionTime(self._updateMarkers)()
        
        
    def _filterMarkersInMainTfFrame(self, markers_rmv: List[MarkerRmv], frames_info:FrameDrawingInfo) -> List[MarkerRmv]:
        """
        Update the markers list to the main TF frame, filtering based on valid transforms.

        Args:
            markers_rmv (List[MarkerRmv]): The list of markers to be updated.

        Returns:
            List[MarkerRmv]: The filtered markers in the main frame.
        """
        filtered_markers = []
        relative_transforms = {frame.name: frame for frame in frames_info}

        for marker in markers_rmv:
            frame = marker.getTfFrame()

            if self.tf_manager.equalMainFrame(frame):
                filtered_markers.append(marker)
                continue

            if frame in relative_transforms and relative_transforms[frame].valid:
                transform = relative_transforms[frame].transform
                transformed_pose = TransformUtils.transformPoseToParentFrame(marker.getPose(), transform)
                if transformed_pose:
                    marker.setFrameId(self.tf_manager.getMainFrame())
                    marker.setPose(transformed_pose)
                    filtered_markers.append(marker)
            
        return filtered_markers

        
    def getNode(self)->Node:
        """
        Method to get the node object.
        Returns:
            Node: The node object.
        """
        return self.node
    