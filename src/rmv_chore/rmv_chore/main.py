import rclpy
from rclpy.node import Node
from time import time, sleep
from typing import List, Dict

from std_msgs.msg import ColorRGBA
from library import (TransformDrawerInfo, TransformGraph, MarkerRmv, TFManager, MarkersHandler, TopicManager, TransformUtils)
from visualization.visualization import Visualization
from library import VisualizationParams


class RMVChoreNode(Node):
    def __init__(self):
        """
        Initialise le nœud RMV Chore, ainsi que ses gestionnaires de TF, marqueurs et visualisation.
        """
        super().__init__("rmv_chore", namespace="rmv")

        self.transform_graph = TransformGraph()
        self.tf_manager = TFManager(self, self.transform_graph)
        self.markers_handler = MarkersHandler(self)

        background_color = ColorRGBA(r=0.1, g=0.1, b=0.1, a=1.0)
        visu_params = VisualizationParams(width=800, height=600, fps=30, background_color=background_color)
        self.visualization = Visualization(self, visu_params, self.transform_graph)
        self.topic_manager = TopicManager(self, self.markers_handler)

        self.create_timer(0.03, self.visualize)
        self.get_logger().info("RMV Chore node initialized successfully.")

    def projectToMainFrame(self, markers: List[MarkerRmv], transforms: List[TransformDrawerInfo]) -> List[MarkerRmv]:
        """
        Filtre les marqueurs en fonction des transformations disponibles.
        """
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
        """
        Met à jour la visualisation avec les marqueurs filtrés.
        """
        start_time = time()
        markers = self.markers_handler.markers
        markers = self.projectToMainFrame(markers, self.transform_graph.getTransformsFromMainFrame())
        self.visualization.visualize(markers)
        # self.get_logger().info(f"Visualization update time: {time() - start_time:.3f}s")


def main():
    rclpy.init()
    node = RMVChoreNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
