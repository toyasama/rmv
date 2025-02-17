import rclpy
from rclpy.node import Node
from time import time, sleep
from typing import List

from std_msgs.msg import ColorRGBA
from library.tf_management.tf import TFManager
from library.topic_management.topic_manager import TopicManager
from library.markers_management.markers import MarkerRmv, MarkersHandler
from library.tf_management.transform_rmv import TransformDrawerInfo
from library.tf_management.transform_utils import TransformUtils
from library.tf_management.graph import TransformGraph
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

        # Timer pour la visualisation (30 FPS ~ 0.03s)
        self.create_timer(0.03, self.visualize)
        self.get_logger().info("RMV Chore node initialized successfully.")

    def filterMarkers(self, markers: List[MarkerRmv], transforms: List[TransformDrawerInfo]) -> List[MarkerRmv]:
        """
        Filtre les marqueurs en fonction des transformations disponibles.
        """
        filtered_markers = []
        for marker in markers:
            if marker.identifier in transforms:
                for transform in transforms:
                    if marker.frame_id == transform.frame:
                        pose_in_main_frame = TransformUtils.transformPoseToParentFrame(marker.pose, transform.pose_in_main_frame)
                        if pose_in_main_frame:
                            marker.modified_pose = pose_in_main_frame
                            filtered_markers.append(marker)
                            break
        return filtered_markers

    def visualize(self):
        """
        Met à jour la visualisation avec les marqueurs filtrés.
        """
        start_time = time()
        markers = self.markers_handler.markers

        # Appliquer un filtrage si nécessaire
        # markers = self.filterMarkers(markers, self.transform_graph.getTransformsFromMainFrame())

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
