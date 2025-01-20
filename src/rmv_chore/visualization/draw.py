import math
from typing import List, Tuple
import cv2
import numpy as np
from markers_management.markers import MarkerRmv
from geometry_msgs.msg import Point
from geometry_msgs.msg import  Transform
from tf_management.tf import TfDrawInfo
from rmv_chore.shared_data import SharedData
class DrawFrames:
    def __init__(self)-> None:
        """
        Constructor for the DrawFrames class.
        """
        self.main_orientation = 0.0
        self.conversion_ratio = 0.0
        self.axis_length_px_m = 0.5
        self.axis_length_px = 0.0
        
    def setMainOrientation(self, orientation: float) -> None:
        """
        Set the orientation of the main frame.
        Args:
            orientation (float): The orientation in radians.
        """
        self.main_orientation = orientation
    
    def setConversionRatio(self, conversion_ratio: float) -> None:
        """
        Set the conversion ratio from meters to pixels.
        Args:
            conversion_ratio (float): The conversion ratio.
        """
        self.conversion_ratio = conversion_ratio
        self.axis_length_px = self.axis_length_px_m * conversion_ratio
        
    def drawMainFrame(self, image: np.ndarray, center: Tuple[int, int],  frame: str) -> None:
        """
        Draw the main reference frame with axes centered on the image.

        Args:
            image (np.ndarray): The image to draw on.
            center (Tuple[int, int]): The center of the main frame in pixels.
            frame (str): The name of the main frame.
        """

        cos_theta = math.cos(self.main_orientation)
        sin_theta = math.sin(self.main_orientation)

        x_axis_end = (
            int(center[0] + self.axis_length_px * cos_theta),
            int(center[1] - self.axis_length_px * sin_theta)
        )

        y_axis_end = (
            int(center[0] - self.axis_length_px * sin_theta),
            int(center[1] - self.axis_length_px * cos_theta)
        )

        self._drawArrowedLine(image, center, x_axis_end, y_axis_end)
        self._drawText(image, frame, (center[0] + 15, center[1] - 15))
        
    def drawFrame(self, image: np.ndarray, frame: Tuple[str, TfDrawInfo], center: Tuple[int, int]) -> None:
        """
        Draw a frame on the image.
        Args:
            image (np.ndarray): The image to draw on.
            frame (List[Tuple[str, Pose]]): The frame to draw.
            center (Tuple[int, int]): The center of the main frame in image coordinates.
        """
            
        position:Transform= frame[1].transform
        x_rel = position.translation.x
        y_rel = position.translation.y
        z = position.rotation.z
        w = position.rotation.w

        relative_orientation = 2 * math.atan2(z, w)
        frame_orientation = self.main_orientation + relative_orientation

        dx = x_rel * self.conversion_ratio
        dy = -y_rel * self.conversion_ratio

        frame_center = (
            int(center[0] + dx * math.cos(self.main_orientation) - dy * math.sin(self.main_orientation)),
            int(center[1] + dx * math.sin(self.main_orientation) + dy * math.cos(self.main_orientation))
        )


        cos_theta = math.cos(frame_orientation)
        sin_theta = math.sin(frame_orientation)

        x_axis_end = (
            int(frame_center[0] + self.axis_length_px * cos_theta),
            int(frame_center[1] - self.axis_length_px * sin_theta)
        )

        y_axis_end = (
            int(frame_center[0] - self.axis_length_px * sin_theta),
            int(frame_center[1] - self.axis_length_px * cos_theta)
        )

        opacity = frame[1].opacity
        self._drawArrowedLine(image, frame_center, x_axis_end, y_axis_end, opacity)
        self._drawText(image, frame[0], (frame_center[0] + 15, frame_center[1] - 15))

    def _drawArrowedLine(self, image: np.ndarray, start: Tuple[int, int], x_axis: Tuple[int, int], y_axis: Tuple[int, int], opacity=1) -> None:
        """
        Draw an arrowed line on the image with a specified opacity.
        Args:
            image (np.ndarray): The image to draw on.
            start (Tuple[int, int]): The start point of the line.
            x_axis (Tuple[int, int]): The end point of the x-axis.
            y_axis (Tuple[int, int]): The end point of the y-axis.
            opacity (float): The transparency of the arrow (0 = fully transparent, 1 = fully opaque).
        """
        opacity = max(0, min(1, opacity))
        overlay = image.copy()

        cv2.arrowedLine(overlay, start, x_axis, color=(0, 0, 255), thickness=2, tipLength=0.1)
        cv2.arrowedLine(overlay, start, y_axis, color=(0, 255, 0), thickness=2, tipLength=0.1)
        
        cv2.addWeighted(overlay, opacity, image, 1 - opacity, 0, image)
        
    def _drawText(self, image: np.ndarray, text: str, position: Tuple[int, int]) -> None:
        """
        Draw text on the image.
        Args:
            image (np.ndarray): The image to draw on.
            text (str): The text to draw.
            position (Tuple[int, int]): The position of the text.
        """
        cv2.putText(image, text, position, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.3,
                    color=(255, 255, 255), thickness=1, lineType=cv2.LINE_AA)
        
class DrawMarkers:
    def __init__(self):
        """
        Constructor for the DrawMarkers class.
        """
        self.conversion_ratio = 0.0
    def setConversionRatio(self, conversion_ratio: float) -> None:
        """
        Set the conversion ratio from meters to pixels.
        Args:
            conversion_ratio (float): The conversion ratio.
        """
        self.conversion_ratio = conversion_ratio
        
    def drawMarkers(self, image : np.ndarray, markers: List[MarkerRmv], center: Tuple[int, int]) -> None:
        """
        Draw markers on the image.
        Args:
            image (np.ndarray): The image to draw on.
            markers (List[MarkerRmv]): The list of markers to draw.
            center (Tuple[int, int]): The center of the main frame in image coordinates.
        """
        for marker in markers:
            position :Point = marker.getPose().position  
            x = position.x
            y = position.y
            px = int(center[0] + x * self.conversion_ratio)
            py = int(center[1] - y * self.conversion_ratio) 

            radius = int(0.1 * self.conversion_ratio) 
            cv2.circle(image, (px, py), radius, (0, 255, 255), -1)  
            
class Draw:
    def __init__(self):
        """
        Constructor for the Draw class.
        """
        self.draw_frames = DrawFrames()
        self.draw_markers = DrawMarkers()
        self.main_orientation = 0.0
        self.conversion_ratio = 0.0

    def setMainOrientation(self, orientation: float) -> None:
        """
        Set the orientation of the main frame.
        """
        self.main_orientation = orientation

    def setConversionRatio(self, conversion_ratio: float) -> None:
        """
        Set the conversion ratio from meters to pixels.
        """
        self.conversion_ratio = conversion_ratio
        self.draw_frames.setConversionRatio(conversion_ratio)
        self.draw_markers.setConversionRatio(conversion_ratio)

    def processVisualization(self, image: np.ndarray, image_center: Tuple[int, int], shared_data: SharedData) -> None:
        """
        Process and draw the visualization on the image.
        """
        self.draw_frames.setMainOrientation(self.main_orientation)

        # Draw main frame
        main_frame = shared_data.get_main_tf()
        if main_frame:
            self.draw_frames.drawMainFrame(image, image_center, main_frame)

        # Draw other frames
        other_frames = shared_data.get_other_tfs()
        for frame in other_frames:
            self.draw_frames.drawFrame(image, frame, image_center)

        # Draw markers
        markers = shared_data.get_markers()
        self.draw_markers.drawMarkers(image, markers, image_center)
