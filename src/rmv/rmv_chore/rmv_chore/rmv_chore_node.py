from pathlib import Path 
from rclpy.node import Node
from rmv_library import (TransformDrawerInfo, TransformGraph, MarkerRmv, TFManager, MarkersHandler, TopicManager, TransformUtils, RmvParameters)
from rmv_visualization.visualization import Visualization
from typing import List, Dict

class RMVChoreNode(Node):
    def __init__(self, publish_on_ros: bool = False):
        super().__init__("rmv_chore", namespace="rmv")
        parent_dir = Path(__file__).resolve().parent.parent
        paremeter_file = parent_dir / "config" / "params.yml"
        self.parameters = RmvParameters(str(paremeter_file))
        self.transform_graph = TransformGraph(self.parameters)
        self.tf_manager = TFManager(self, self.transform_graph)
        self.markers_handler = MarkersHandler(self)

        self.visualization = Visualization(self, self.parameters, self.transform_graph, publish_on_ros)
        self.topic_manager = TopicManager(self, self.markers_handler)
        self.period = 1/self.parameters.visualization.fps
        self.create_timer(self.period, self.visualize)
        self.get_logger().info("RMV Chore node initialized successfully.")
        

    def projectToMainFrame(self, markers: List[MarkerRmv], transforms: List[TransformDrawerInfo]) -> List[MarkerRmv]:
        main_frame = self.transform_graph.main_frame
        transform_dict: Dict[str, TransformDrawerInfo] = {transform.transform_name: transform.pose_in_main_frame for transform in transforms}
        projected_markers = []
        for marker in markers:
            if marker.frame_id == main_frame:
                marker.modified_pose = marker.pose
                projected_markers.append(marker)
            elif marker.frame_id in transform_dict:
                pose_in_main_frame = TransformUtils.transformPoseToParentFrame(marker.pose, transform_dict[marker.frame_id])
                if pose_in_main_frame:
                    marker.modified_pose = pose_in_main_frame
                    projected_markers.append(marker)
        return projected_markers

    def visualize(self):
        markers = self.markers_handler.markers
        markers = self.projectToMainFrame(markers, self.transform_graph.getTransformsFromMainFrame())
        self.visualization.visualize(markers)
        
        
