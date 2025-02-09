from .markers import MarkerRmv
from visualization_msgs.msg import Marker
from typing import List
from rclpy.node import Node
from ..topic_management.topic_manager import TopicManager
from ..parameters.params import RmvParams , VisualizationParams
from threading import Thread, RLock
from time import sleep
from ..tf_management.tf import TFManager, FrameDrawingInfo
from ..tf_management.transform import TransformUtils
from ..utils.shared_data import SharedData
from ..utils.timer_log import TimerLogger
from std_msgs.msg import ColorRGBA
import copy

    

class dataManager(Thread):
    def __init__(self, node: Node)-> None:
        """
        Constructor for the DataManager class.
        Args:
            node (Node): The ROS2 node.
        """
        super().__init__()
        self.__shared_data_lock = RLock()
        self.__running = True
        self._shared_data = SharedData()
        rmv_params: RmvParams = RmvParams()
        background_color = ColorRGBA(r=0.1, g=0.1, b=0.1, a=1.0)
        self.visu_params = VisualizationParams(width=800, height=600, fps=30, background_color=background_color)
        
        self.node:Node = node
        self.markers_manager: TopicManager= TopicManager(self.node, rmv_params.UPDATE_MARKERS_PROCESS_PERIOD)
       
        self.tf_manager = TFManager(self.node)
        self.main_frame_frame_info: FrameDrawingInfo = FrameDrawingInfo()
        self.timer_logger_tf = TimerLogger(self.node, 2.0)
    
    def run(self):
        """
        Run the data manager thread.
        """
        while self.__running:
            self.timer_logger_tf.logExecutionTime(self.process)()
            sleep(1/self.visu_params.fps)
    def process(self):
        """
        Process the data manager.
        """
        markers_rmv = copy.deepcopy(self.markers_manager.markers)
        self.tf_manager.updateAllTransformsFrom()
        self.tf_manager.setDefaultMainFrame() 
        self.main_frame_frame_info  = self.tf_manager.getMainFrame()
        
        relative_transforms:dict[str,FrameDrawingInfo] = self.tf_manager.getAllTransformsFromMainFrame()
        filtered_markers = self._filterMarkersInMainTfFrame(markers_rmv, relative_transforms)
        with self.__shared_data_lock:
            self._shared_data.update_markers(filtered_markers)
            self._shared_data.update_main_tf(self.main_frame_frame_info)
            self._shared_data.update_other_tfs( list(relative_transforms.values()))
        
    
    def getSharedData(self)->SharedData:
        """
        Get the shared data object.
        Returns:
            SharedData: The shared data object.
        """
        with self.__shared_data_lock:
            return self._shared_data
    
    def _filterMarkersInMainTfFrame(self, markers_rmv: List[MarkerRmv], relative_transforms:dict[str,FrameDrawingInfo]) -> List[MarkerRmv]:
        """
        Update the markers list to the main TF frame, filtering based on valid transforms.

        Args:
            markers_rmv (List[MarkerRmv]): The list of markers to be updated.

        Returns:
            List[MarkerRmv]: The filtered markers in the main frame.
        """
        filtered_markers = []
        for marker in markers_rmv:
            frame = marker.getTfFrame()
            if not self.main_frame_frame_info.name:
                break
            if self.main_frame_frame_info.name == frame :
                filtered_markers.append(marker)
                continue

            if frame in relative_transforms :
                transform = relative_transforms[frame].transform
                transformed_pose = TransformUtils.transformPoseToParentFrame(marker.getPose(), transform)
                if transformed_pose:
                    marker.new_frame_id = self.main_frame_frame_info.name
                    marker.pose_in_new_frame = transformed_pose
                    filtered_markers.append(marker)
            # else:
            #     print("not valid")
            
        return filtered_markers
