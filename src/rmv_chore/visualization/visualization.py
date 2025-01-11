import cv2  # OpenCV for drawing on images
import threading
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge
import numpy as np
from typing import List, Tuple
from rclpy.node import Node
from visualization.queue_list import QueueList
from markers_management.markers import MarkerRmv
import rclpy
import time
from geometry_msgs.msg import Point

class VisualizationParams:
    """
    Parameters for the Visualization class.
    """
    def __init__(self, width: int, height: int, fps: int, background_color: ColorRGBA):
        self.width = width
        self.height = height
        self.fps = fps
        self.background_color = background_color
        self.conversion_ratio = 1.0 / 0.05  # 1 meter in real life equals 10 pixels in the image


class Visualization(threading.Thread):
    def __init__(self, node, params: VisualizationParams, publish_image: bool = False, queue_list: QueueList = None):
        """
        Constructor for the Visualization class.

        Args:
            node: The ROS 2 node for publishing images.
            params (VisualizationParams): Visualization parameters.
            publish_image (bool): Whether to publish the image on a ROS topic.
            queue_list (QueueList): Queue list to receive data from RmvChore.
        """
        super().__init__()
        self.node: Node = node
        self.params = params
        self.publish_image = publish_image
        self.bridge = CvBridge()

        self.image = self.createNewImage()
        self.publisher = None
        if self.publish_image:
            self.publisher = self.node.create_publisher(Image, "visualization_image", 10)
        self.running = False
        self.queue_list: QueueList = queue_list

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

    def clearImage(self) -> None:
        """
        Clear the current image by resetting it to the background color.
        """
        self.image = self.createNewImage()

    def drawMainRepere(self, center: Tuple[int, int]) -> None:
        """
        Draw the main reference frame at the center of the image.

        Args:
            center (Tuple[int, int]): The center of the main reference frame in image coordinates.
        """
        x_length = int(10 * self.params.conversion_ratio)  # 10 cm in image units
        y_length = x_length

        # Draw x-axis (blue)
        cv2.arrowedLine(self.image, center, (center[0], center[1] - x_length), (255, 0, 0), 2)
        # Draw y-axis (green)
        cv2.arrowedLine(self.image, center, (center[0]- y_length, center[1]), (0, 255, 0), 2)

    def drawRepere(self, position: Tuple[float, float, float], center: Tuple[int, int]) -> None:
        """
        Draw a secondary reference frame relative to the main one.

        Args:
            position (Tuple[float, float, float]): The position of the secondary frame in meters relative to the main one.
            center (Tuple[int, int]): The center of the main reference frame in image coordinates.
        """
        x, y, z = position
        px = int(center[0] + x * self.params.conversion_ratio)
        py = int(center[1] - y * self.params.conversion_ratio)  # Negative because image origin is top-left

        self.drawMainRepere((px, py))

    def drawMarkers(self, markers: List[MarkerRmv], center: Tuple[int, int]) -> None:
        """
        Draw markers as circles on the image.

        Args:
            markers (List[MarkerRmv]): List of markers to draw.
            center (Tuple[int, int]): Center of the main reference frame in image coordinates.
        """
        for marker in markers:
            position :Point = marker.getPose().position  # Assuming marker has a method to get its (x, y, z) position
            x = position.x
            y = position.y
            px = int(center[0] + x * self.params.conversion_ratio)
            py = int(center[1] - y * self.params.conversion_ratio)  # Negative for image y-axis inversion

            radius = int(0.1 * self.params.conversion_ratio)  # 10 cm in image units
            cv2.circle(self.image, (px, py), radius, (0, 255, 255), -1)  # Yellow circle

    def run(self) -> None:
        """
        The main process of the thread to draw and optionally publish the image.
        """
        self.running = True
        interval = 1.0 / self.params.fps
        next_iteration_time = time.time()

        while self.running:
            if not rclpy.ok():
                print("Shutting down visualization thread")
                break
            
            self.clearImage()

            # Draw main reference frame
            if not self.queue_list.mainTfEmpty():
                center = (self.params.width // 2, self.params.height // 2)
                self.drawMainRepere(center)
                # Draw other markers and references
                if not self.queue_list.markersEmpty():
                    filtered_markers = self.queue_list.getMarkers()
                    self.drawMarkers(filtered_markers, center)

            if self.publish_image and self.publisher:
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
                    self.publisher.publish(ros_image)
                except Exception as e:
                    self.node.get_logger().error(f"Failed to publish image: {str(e)}")

            now = time.time()
            sleep_duration = max(0, next_iteration_time - now)
            time.sleep(sleep_duration)
            next_iteration_time += interval

    def stop(self) -> None:
        """
        Stop the thread gracefully.
        """
        self.running = False
