from .markers import MarkerRmv
from visualization_msgs.msg import Marker
from typing import List
from rclpy.node import Node
from ..topic_management.topic_manager import TopicManager
from ..parameters.params import RmvParams , VisualizationParams
from threading import Thread, RLock
from time import sleep
from ..tf_management.tf import TFManager, FrameDrawingInfo, TransformUtils
from ..utils.shared_data import SharedData
from std_msgs.msg import ColorRGBA
import copy

class MarkersManager(Thread):
    def __init__(self,node, rmv_params: RmvParams)->None:
        """
        Constructor for the MarkersManager class.
        Args:
            node (Node): The node object.
        """
        super().__init__()
        self.markers_list:dict[tuple[str, int],MarkerRmv] = {}
        self.node:Node = node
        self.topic_manager : TopicManager = TopicManager(self.node, rmv_params.UPDATE_TOPIC_PROCESS_PERIOD)
        self.__lock_markers_list = RLock()
        self.__running = True
        
    def _addMarker(self, marker:Marker)->None:
        """
        Add a new marker to the markers list.

        Args:
            marker (Marker): The marker to be added.
        """
        new_marker = MarkerRmv(marker, self.node.get_clock().now().to_msg())
        with self.__lock_markers_list:
            self.markers_list[new_marker.getIdentifier()] = new_marker
        
    def run(self)->None:
        """
        Process a list of new markers.
        Args:
            markers (list[Marker]): The list of markers to be processed.
        """
        while self.__running:
            markers = self.topic_manager.extractMarkersList()
            for marker in markers:
                self._addMarker(marker)
            self._deleteExpiredMarkers()
            sleep(0.1)
        
    def _deleteExpiredMarkers(self):
        """
        Delete the expired markers from the markers list.
        """
        current_time = self.node.get_clock().now().to_msg()
        with self.__lock_markers_list:
            self.markers_list = {
                identifier: marker
                for identifier, marker in self.markers_list.items()
                if not marker.isExpired(current_time)
            }
        
    def getMarkersList(self)->List[MarkerRmv]:
        """
        Get the markers list.

        Returns:
            List[MarkerRmv]: The markers list.
        """
        with self.__lock_markers_list:
            return list(self.markers_list.values())
    
    def _cleanMarkersList(self):
        """
        Clean the markers list.
        """
        with self.__lock_markers_list:
            self.markers_list.clear()
    

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
        self.markers_manager: MarkersManager= MarkersManager(self.node, rmv_params)
        self.markers_manager.start()        
        self.tf_manager = TFManager(self.node, rmv_params.TIMEOUT_TF_BUFFER)
        self.tf_manager.start()
        self.main_frame_frame_info: FrameDrawingInfo = FrameDrawingInfo()
    
    def run(self):
        """
        Run the data manager thread.
        """
        while self.__running:
            
            markers_rmv = copy.deepcopy(self.markers_manager.getMarkersList())
            self.tf_manager.setDefaultMainFrame() 
            self.main_frame_frame_info  = self.tf_manager.getMainFrame()
            
            relative_transforms:dict[str,FrameDrawingInfo] = self.tf_manager.getAllTransformsFromMainFrame()
            filtered_markers = self._filterMarkersInMainTfFrame(markers_rmv, relative_transforms)
            with self.__shared_data_lock:
                self._shared_data.update_markers(filtered_markers)
                self._shared_data.update_main_tf(self.main_frame_frame_info)
                self._shared_data.update_other_tfs( list(relative_transforms.values()))
            sleep(1/self.visu_params.fps)
    
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
                    marker.setFrameId(self.main_frame_frame_info.name)
                    marker.setPose(transformed_pose)
                    filtered_markers.append(marker)
            # else:
            #     print("not valid")
            
        return filtered_markers
