import numpy as np
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from typing import  Tuple
from rclpy.node import Node
import time
from visualization.draw import Draw
from rmv_chore.shared_data import SharedData
from parameters.params import VisualizationParams
from rclpy.qos import qos_profile_sensor_data

class Visualization():
    def __init__(self, node: Node, params: VisualizationParams, shared_data: SharedData):
        """
        Constructor for the Visualization class.
        args:
            node (Node): The ROS2 node.
            params (VisualizationParams): The parameters for the visualization.
            shared_data (SharedData): The shared data.
        """
        super().__init__()
        self.node: Node = node
        self.params = params
        self.shared_data = shared_data
        self.bridge = CvBridge()
        self.image = self.createNewImage()
        self.publisher = self.node.create_publisher(Image, "visualization_image", qos_profile_sensor_data)
        self.draw = Draw()
        
    def run(self):
        """
        Run the visualization thread.
        """
        
        self.clearImage()

        main_tf = self.shared_data.get_main_tf()

        if main_tf:
            self.draw.setConversionRatio(self.params.conversion_ratio)
            self.draw.processVisualization(self.image, self.getImageCenter(), self.shared_data)

        if self.params.publish_image:
            ros_image = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
            self.publisher.publish(ros_image)


    def getImageCenter(self) -> Tuple[int, int]:
        """
        Get the center of the image.

        Returns:
            Tuple[int, int]: The center coordinates (x, y).
        """
        return (self.params.width // 2, self.params.height // 2)
    
    def clearImage(self) -> None:
        """
        Clear the current image by resetting it to the background color.
        """
        self.image = self.createNewImage()

    def createNewImage(self) -> np.ndarray:
        """
        Create a new blank image with the specified background color.
        Returns:
            np.ndarray: The created image.
        """
        bg_color = (
            int(self.params.background_color.r * 255),
            int(self.params.background_color.g * 255),
            int(self.params.background_color.b * 255),
        )
        return np.full(
            (self.params.height, self.params.width, 3), bg_color, dtype=np.uint8
        )