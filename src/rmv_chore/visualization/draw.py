import math
from typing import List, Tuple
import cv2
import numpy as np
from markers_management.markers import MarkerRmv
from geometry_msgs.msg import Point, Transform
from tf_management.tf import FrameDrawingInfo
from rmv_chore.shared_data import SharedData
from visualization_msgs.msg import Marker

class DrawingUtils:
    @staticmethod
    def draw_arrow(image: np.ndarray, start: Tuple[int, int], end: Tuple[int, int], color: Tuple[int, int, int], thickness: int = 2, tip_length: float = 0.1, opacity: float = 1.0) -> None:
        overlay = image.copy()
        cv2.arrowedLine(overlay, start, end, color=color, thickness=thickness, tipLength=tip_length)
        cv2.addWeighted(overlay, opacity, image, 1 - opacity, 0, image)

    @staticmethod
    def draw_text(image: np.ndarray, text: str, position: Tuple[int, int], font_scale: float = 0.3, color: Tuple[int, int, int] = (255, 255, 255), thickness: int = 1) -> None:
        cv2.putText(image, text, position, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=font_scale,
                    color=color, thickness=thickness, lineType=cv2.LINE_AA)

    @staticmethod
    def draw_circle(image: np.ndarray, center: Tuple[int, int], radius: int, color: Tuple[int, int, int], opacity: float = 1.0) -> None:
        overlay = image.copy()
        cv2.circle(overlay, center, radius, color, -1)
        cv2.addWeighted(overlay, opacity, image, 1 - opacity, 0, image)

    @staticmethod
    def draw_rectangle(image: np.ndarray, top_left: Tuple[int, int], bottom_right: Tuple[int, int], color: Tuple[int, int, int], opacity: float = 1.0) -> None:
        overlay = image.copy()
        cv2.rectangle(overlay, top_left, bottom_right, color, -1)
        cv2.addWeighted(overlay, opacity, image, 1 - opacity, 0, image)

class DrawFrames:
    def __init__(self) -> None:
        """
        Constructor for the DrawFrames class.
        """
        self.main_orientation = 0.0
        self.conversion_ratio = 0.0
        self.axis_length_px_m = 0.5
        self.axis_length_px = 0.0

    def setMainOrientation(self, orientation: float) -> None:
        """
        Set the main orientation for the frames.
        args:
            orientation (float): The main orientation.
        """
        self.main_orientation = orientation

    def setConversionRatio(self, conversion_ratio: float) -> None:
        """
        Set the conversion ratio for the frames.
        args:
            conversion_ratio (float): The conversion ratio.
        """
        self.conversion_ratio = conversion_ratio
        self.axis_length_px = self.axis_length_px_m * conversion_ratio

    def drawMainFrame(self, image: np.ndarray, center: Tuple[int, int], frame_name: str) -> None:
        """
        Draw the main frame on the image.
        args:
            image (np.ndarray): The image to draw on.
            center (Tuple[int, int]): The center of the image.
            frame_name (str): The name of the frame.
        """
        self._drawAxes(image, center, self.main_orientation)
        DrawingUtils.draw_text(image, frame_name, (center[0] + 15, center[1] - 15))

    def drawFrame(self, image: np.ndarray, frame: FrameDrawingInfo, center: Tuple[int, int]) -> None:
        """
        Draw a frame on the image.
        args:
            image (np.ndarray): The image to draw on.
            frame (FrameDrawingInfo): The frame information.
            center (Tuple[int, int]): The center of the image.
        """
        position: Transform = frame.transform
        relative_orientation = 2 * math.atan2(position.rotation.z, position.rotation.w)
        frame_orientation = self.main_orientation + relative_orientation

        frame_center = self._transformPoint(position, center)

        self._drawAxes(image, frame_center, frame_orientation, frame.opacity)
        DrawingUtils.draw_text(image, frame.name, (frame_center[0] + 15, frame_center[1] - 15))

        if frame.start_connection and frame.end_connection:
            start_point = self._transformPoint(frame.start_connection, center)
            end_point = self._transformPoint(frame.end_connection, center)
            DrawingUtils.draw_arrow(image, start_point, end_point, color=(255, 255, 255), opacity=frame.opacity)

    def _transformPoint(self, transform: Transform, center: Tuple[int, int]) -> Tuple[int, int]:
        """
        Transform a point from the frame to the image.
        """
        dx = transform.translation.x * self.conversion_ratio
        dy = -transform.translation.y * self.conversion_ratio
        return (
            int(center[0] + dx * math.cos(self.main_orientation) - dy * math.sin(self.main_orientation)),
            int(center[1] + dx * math.sin(self.main_orientation) + dy * math.cos(self.main_orientation))
        )

    def _drawAxes(self, image: np.ndarray, frame_center: Tuple[int, int], frame_orientation: float, opacity: float = 1.0) -> None:
        """
        Draw the axes of the frame.
        args:
            image (np.ndarray): The image to draw on.
            frame_center (Tuple[int, int]): The center of the frame.
            frame_orientation (float): The orientation of the frame.
            opacity (float): The opacity of the frame.
        """
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
        DrawingUtils.draw_arrow(image, frame_center, x_axis_end, color=(0, 0, 255), opacity=opacity)
        DrawingUtils.draw_arrow(image, frame_center, y_axis_end, color=(0, 255, 0), opacity=opacity)

class DrawMarkers:
    def __init__(self):
        self.conversion_ratio = 0.0

    def setConversionRatio(self, conversion_ratio: float) -> None:
        self.conversion_ratio = conversion_ratio

    def drawMarkers(self, image: np.ndarray, markers: List[MarkerRmv], center: Tuple[int, int]) -> None:
        sorted_markers = sorted(markers, key=lambda m: m.color.a)
        for marker in sorted_markers:
            position: Point = marker.getPose().position
            px = int(center[0] + position.x * self.conversion_ratio)
            py = int(center[1] - position.y * self.conversion_ratio)

            color = (
                int(marker.color.r * 255),
                int(marker.color.g * 255),
                int(marker.color.b * 255)
            )

            if marker.getType() == Marker.ARROW:
                end_point = (px + 20, py)  # Example: arbitrary endpoint calculation
                DrawingUtils.draw_arrow(image, (px, py), end_point, color, opacity=marker.color.a)

            elif marker.type == Marker.CUBE:
                side_length = int(marker.scale.x * self.conversion_ratio)
                top_left = (px - side_length // 2, py - side_length // 2)
                bottom_right = (px + side_length // 2, py + side_length // 2)
                DrawingUtils.draw_rectangle(image, top_left, bottom_right, color, opacity=marker.color.a)

            elif marker.type == Marker.SPHERE:
                radius = int(marker.scale.x * self.conversion_ratio / 2)
                DrawingUtils.draw_circle(image, (px, py), radius, color, opacity=marker.color.a)

            elif marker.type == Marker.TEXT_VIEW_FACING:
                # DrawingUtils.draw_text(image, marker.text, (px, py), font_scale=0.5, color=color)
                pass

class Draw:
    def __init__(self):
        """
        Constructor for the Draw class.
        """
        self.drawFrames = DrawFrames()
        self.drawMarkers = DrawMarkers()
        self.main_orientation = 0.0
        self.conversion_ratio = 0.0

    def setMainOrientation(self, orientation: float) -> None:
        """
        Set the main orientation for the visualization.
        args:
            orientation (float): The main orientation.
        """
        self.main_orientation = orientation
        self.drawFrames.setMainOrientation(orientation)

    def setConversionRatio(self, conversion_ratio: float) -> None:
        """
        Set the conversion ratio for the visualization.
        args:
            conversion_ratio (float): The conversion ratio.
        """
        self.conversion_ratio = conversion_ratio
        self.drawFrames.setConversionRatio(conversion_ratio)
        self.drawMarkers.setConversionRatio(conversion_ratio)

    def processVisualization(self, image: np.ndarray, image_center: Tuple[int, int], shared_data: SharedData) -> None:
        """
        Process the visualization of the frames and markers.
        args:
            image (np.ndarray): The image to draw on.
            image_center (Tuple[int, int]): The center of the image.
            shared_data (SharedData): The shared data.
        """
        main_frame = shared_data.get_main_tf()
        if main_frame:
            self.drawFrames.drawMainFrame(image, image_center, main_frame)

        for frame in shared_data.get_other_tfs():
            self.drawFrames.drawFrame(image, frame, image_center)

        markers_list = shared_data.get_markers()
        if markers_list:
            self.drawMarkers.drawMarkers(image, markers_list, image_center)
        
