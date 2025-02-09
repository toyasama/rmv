from .markers import MarkerRmv
from visualization_msgs.msg import Marker
from typing import List
from rclpy.node import Node
from ..topic_management.topic_manager import TopicManager
from ..parameters.params import RmvParams , VisualizationParams
from threading import Thread, RLock
from time import sleep
from ..tf_management.tf import TFManager, FrameRMV
from ..tf_management.transform_utils import TransformUtils
from ..utils.shared_data import SharedData
from ..utils.timer_log import TimerLogger
from std_msgs.msg import ColorRGBA
import copy

    

class dataManager():
    def __init__(self, node: Node)-> None:
        """
        Constructor for the DataManager class.
        Args:
            node (Node): The ROS2 node.
        """
        super().__init__()
        self._shared_data = SharedData()
        self.markers_manager: TopicManager= TopicManager(node, 0.1)
       
        self.tf_manager = TFManager(node)
        self.main_frame_frame_info: FrameRMV = FrameRMV()
        self.timer_logger_tf = TimerLogger(node, 2.0)
    
    def processData(self):
        """
        Run the data manager thread.
        """
        self.timer_logger_tf.logExecutionTime(self._process)()
    
    def _process(self):
        """
        Process the data manager.
        """
        markers_rmv = self.markers_manager.markers
        self.tf_manager.updateAllTransformsFrom()
        self.tf_manager.setDefaultMainFrame() 
        self.main_frame_frame_info  = self.tf_manager.getMainFrame()
        
        relative_transforms:dict[str,FrameRMV] = self.tf_manager.getAllTransformsFromMainFrame()
        filtered_markers = self._filterMarkersInMainTfFrame(markers_rmv, relative_transforms)

        self._shared_data.update_markers(filtered_markers)
        self._shared_data.update_main_tf(self.main_frame_frame_info)
        self._shared_data.update_other_tfs( list(relative_transforms.values()))
    
    @property
    def shared_data(self)->SharedData:
        """
        Get the shared data object.

        Returns:
            SharedData: The shared data object.
        """
        return self._shared_data
    
    def _filterMarkersInMainTfFrame(self, markers_rmv: List[MarkerRmv], relative_transforms:dict[str,FrameRMV]) -> List[MarkerRmv]:
        """
        Update the markers list to the main TF frame, filtering based on valid transforms.

        Args:
            markers_rmv (List[MarkerRmv]): The list of markers to be updated.

        Returns:
            List[MarkerRmv]: The filtered markers in the main frame.
        """
        filtered_markers = []
        for marker in markers_rmv:
            frame = marker.frame_id
            if not self.main_frame_frame_info.name:
                break
            if self.main_frame_frame_info.name == frame :
                marker.modified_pose = marker.pose
                filtered_markers.append(marker)
                continue

            if frame in relative_transforms :
                transform = relative_transforms[frame].transform
                transformed_pose = TransformUtils.transformPoseToParentFrame(marker.pose, transform)
                if transformed_pose:
                    marker.modified_pose = transformed_pose
                    filtered_markers.append(marker)
        return filtered_markers
