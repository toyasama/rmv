import numpy as np
import cv2
import math
from typing import List, Tuple
from geometry_msgs.msg import Point, Transform
from visualization_msgs.msg import Marker
from tf_management.graph import FrameDrawingInfo
from rmv_chore.shared_data import SharedData


class DrawingUtils:
    @staticmethod
    def _apply_roi(image: np.ndarray, roi: Tuple[int, int, int, int]) -> np.ndarray:
        """
        Extract the Region of Interest (ROI) from the image.
        Args:
            image (np.ndarray): The input image.
            roi (Tuple[int, int, int, int]): The region of interest as (x, y, width, height).
        Returns:
            np.ndarray: The cropped image for the ROI.
        """
        x, y, w, h = roi
        return image[y:y+h, x:x+w]

    @staticmethod
    def draw_arrow(image: np.ndarray, start: Tuple[int, int], end: Tuple[int, int], color: Tuple[int, int, int], thickness: int = 2, tip_length: float = 0.1, opacity: float = 1.0, roi: Tuple[int, int, int, int] = None) -> None:
        """
        Draw an arrow with optional opacity and ROI.
        """
        if roi:
            x, y, _, _ = roi
            start = (start[0] - x, start[1] - y)
            end = (end[0] - x, end[1] - y)
            cropped_image = DrawingUtils._apply_roi(image, roi)
            if opacity < 1.0:
                overlay = cropped_image.copy()
                cv2.arrowedLine(overlay, start, end, color=color, thickness=thickness, tipLength=tip_length)
                cv2.addWeighted(overlay, opacity, cropped_image, 1 - opacity, 0, cropped_image)
            else:
                cv2.arrowedLine(cropped_image, start, end, color=color, thickness=thickness, tipLength=tip_length)
        else:
            if opacity < 1.0:
                overlay = image.copy()
                cv2.arrowedLine(overlay, start, end, color=color, thickness=thickness, tipLength=tip_length)
                cv2.addWeighted(overlay, opacity, image, 1 - opacity, 0, image)
            else:
                cv2.arrowedLine(image, start, end, color=color, thickness=thickness, tipLength=tip_length)

    @staticmethod
    def draw_text(image: np.ndarray, text: str, position: Tuple[int, int], font_scale: float = 0.3, color: Tuple[int, int, int] = (255, 255, 255), thickness: int = 1, roi: Tuple[int, int, int, int] = None) -> None:
        """
        Draw text on an image with optional ROI.
        """
        if roi:
            x, y, _, _ = roi
            position = (position[0] - x, position[1] - y)
            cropped_image = DrawingUtils._apply_roi(image, roi)
            cv2.putText(cropped_image, text, position, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=font_scale,
                        color=color, thickness=thickness, lineType=cv2.LINE_AA)
        else:
            cv2.putText(image, text, position, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=font_scale,
                        color=color, thickness=thickness, lineType=cv2.LINE_AA)

    @staticmethod
    def draw_circle(image: np.ndarray, center: Tuple[int, int], radius: int, color: Tuple[int, int, int], opacity: float = 1.0, roi: Tuple[int, int, int, int] = None) -> None:
        """
        Draw a circle with optional opacity and ROI.
        """
        if roi:
            x, y, _, _ = roi
            center = (center[0] - x, center[1] - y)
            cropped_image = DrawingUtils._apply_roi(image, roi)
            if opacity < 1.0:
                overlay = cropped_image.copy()
                cv2.circle(overlay, center, radius, color, -1)
                cv2.addWeighted(overlay, opacity, cropped_image, 1 - opacity, 0, cropped_image)
            else:
                cv2.circle(cropped_image, center, radius, color, -1)
        else:
            if opacity < 1.0:
                overlay = image.copy()
                cv2.circle(overlay, center, radius, color, -1)
                cv2.addWeighted(overlay, opacity, image, 1 - opacity, 0, image)
            else:
                cv2.circle(image, center, radius, color, -1)

    @staticmethod
    def draw_rectangle(image: np.ndarray, top_left: Tuple[int, int], bottom_right: Tuple[int, int], color: Tuple[int, int, int], opacity: float = 1.0, roi: Tuple[int, int, int, int] = None) -> None:
        """
        Draw a rectangle with optional opacity and ROI.
        """
        if roi:
            x, y, _, _ = roi
            top_left = (top_left[0] - x, top_left[1] - y)
            bottom_right = (bottom_right[0] - x, bottom_right[1] - y)
            cropped_image = DrawingUtils._apply_roi(image, roi)
            if opacity < 1.0:
                overlay = cropped_image.copy()
                cv2.rectangle(overlay, top_left, bottom_right, color, -1)
                cv2.addWeighted(overlay, opacity, cropped_image, 1 - opacity, 0, cropped_image)
            else:
                cv2.rectangle(cropped_image, top_left, bottom_right, color, -1)
        else:
            if opacity < 1.0:
                overlay = image.copy()
                cv2.rectangle(overlay, top_left, bottom_right, color, -1)
                cv2.addWeighted(overlay, opacity, image, 1 - opacity, 0, image)
            else:
                cv2.rectangle(image, top_left, bottom_right, color, -1)

    @staticmethod
    def isPointInImage(point: Tuple[int, int], img_width: int, img_height: int) -> bool:
        """
        Check if a point is within the image boundaries.
        Args:
            point (Tuple[int, int]): The (x, y) coordinates of the point.
            img_width (int): Width of the image.
            img_height (int): Height of the image.
        Returns:
            bool: True if the point is inside the image boundaries, False otherwise.
        """
        px, py = point
        return 0 <= px < img_width and 0 <= py < img_height

class DrawFrames:
    def __init__(self) -> None:
        self.main_orientation = 0.0
        self.cos_main_orientation = 1.0
        self.sin_main_orientation = 0.0
        self.conversion_ratio = 0.0
        self.axis_length_px_m = 0.5
        self.axis_length_px = 0.0

    def setMainOrientation(self, orientation: float) -> None:
        """
        Precompute cosine and sine values for the main orientation.
        """
        self.main_orientation = orientation
        self.cos_main_orientation = math.cos(orientation)
        self.sin_main_orientation = math.sin(orientation)

    def setConversionRatio(self, conversion_ratio: float) -> None:
        self.conversion_ratio = conversion_ratio
        self.axis_length_px = self.axis_length_px_m * conversion_ratio

    def drawMainFrame(self, image: np.ndarray, center: Tuple[int, int], frame_name: str) -> None:
        self._drawAxes(image, center, self.main_orientation)
        DrawingUtils.draw_text(image, frame_name, (center[0] + 15, center[1] - 15))

    def drawFrame(self, image: np.ndarray, frame_info: FrameDrawingInfo, center: Tuple[int, int]) -> None:
        frame_orientation = self.main_orientation + 2 * math.atan2(frame_info.transform.rotation.z, frame_info.transform.rotation.w)
        frame_center = self._transformPoint(frame_info.transform, center)

        self._drawAxes(image, frame_center, frame_orientation)
        DrawingUtils.draw_text(image, "Frame", (frame_center[0] + 15, frame_center[1] - 15))

        if frame_info.start_connection and frame_info.end_connection:
            start_point = self._transformPoint(frame_info.start_connection, center)
            end_point = self._transformPoint(frame_info.end_connection, center)
            DrawingUtils.draw_arrow(image, start_point, end_point, color=(255, 255, 255), opacity=frame_info.opacity)

    def _transformPoint(self, transform: Transform, center: Tuple[int, int]) -> Tuple[int, int]:
        """
        Transform a point using precomputed cos/sin values and the conversion ratio.
        """
        dx = transform.translation.x * self.conversion_ratio
        dy = -transform.translation.y * self.conversion_ratio
        return (
            int(center[0] + dx * self.cos_main_orientation - dy * self.sin_main_orientation),
            int(center[1] + dx * self.sin_main_orientation + dy * self.cos_main_orientation)
        )

    def _drawAxes(self, image: np.ndarray, frame_center: Tuple[int, int], frame_orientation: float) -> None:
        """
        Draw X and Y axes of a frame.
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

        DrawingUtils.draw_arrow(image, frame_center, x_axis_end, color=(0, 0, 255))
        DrawingUtils.draw_arrow(image, frame_center, y_axis_end, color=(0, 255, 0))


class DrawMarkers:
    def __init__(self):
        self.conversion_ratio = 0.0

    def setConversionRatio(self, conversion_ratio: float) -> None:
        self.conversion_ratio = conversion_ratio

    def drawMarkers(self, image: np.ndarray, markers: List[Marker], center: Tuple[int, int]) -> None:
        """
        Efficiently draw markers by precomputing positions and sorting them.
        """
        for marker in sorted(markers, key=lambda m: m.color.a):
            position: Point = marker.pose.position
            px = int(center[0] + position.x * self.conversion_ratio)
            py = int(center[1] - position.y * self.conversion_ratio)

            color = (
                int(marker.color.r * 255),
                int(marker.color.g * 255),
                int(marker.color.b * 255)
            )

            if marker.type == Marker.ARROW:
                end_point = (px + 20, py)
                DrawingUtils.draw_arrow(image, (px, py), end_point, color, opacity=marker.color.a)

            elif marker.type == Marker.CUBE:
                side_length = int(marker.scale.x * self.conversion_ratio)
                top_left = (px - side_length // 2, py - side_length // 2)
                bottom_right = (px + side_length // 2, py + side_length // 2)
                DrawingUtils.draw_rectangle(image, top_left, bottom_right, color, opacity=marker.color.a)

            elif marker.type == Marker.SPHERE:
                radius = int(marker.scale.x * self.conversion_ratio / 2)
                DrawingUtils.draw_circle(image, (px, py), radius, color, opacity=marker.color.a)


class Draw:
    def __init__(self):
        self.drawFrames = DrawFrames()
        self.drawMarkers = DrawMarkers()

    def setMainOrientation(self, orientation: float) -> None:
        self.drawFrames.setMainOrientation(orientation)

    def setConversionRatio(self, conversion_ratio: float) -> None:
        self.drawFrames.setConversionRatio(conversion_ratio)
        self.drawMarkers.setConversionRatio(conversion_ratio)

    def processVisualization(self, image: np.ndarray, image_center: Tuple[int, int], shared_data: SharedData) -> None:
        main_frame = shared_data.get_main_tf()
        other_frames = shared_data.get_other_tfs()
        markers_list = shared_data.get_markers()
        
        other_frames, markers_list = self.filterElementsInView(image, image_center, other_frames, markers_list)

        if main_frame:
            self.drawFrames.drawMainFrame(image, image_center, main_frame)

        for frame in other_frames:
            self.drawFrames.drawFrame(image, frame, image_center)

        if markers_list:
            self.drawMarkers.drawMarkers(image, markers_list, image_center)
        
    
    
    def filterElementsInView(self, image: np.ndarray, image_center: Tuple[int, int], 
                                frames: List[FrameDrawingInfo], markers: List[Marker]) -> Tuple[List[FrameDrawingInfo], List[Marker]]:
        """
        Filter markers and TFs to include only those visible within the image.
        Args:
            image (np.ndarray): The image to check boundaries against.
            image_center (Tuple[int, int]): The center of the image in pixel coordinates.
            frames (List[FrameDrawingInfo]): List of frame transformations to filter.
            markers (List[Marker]): List of markers to filter.
        Returns:
            Tuple[List[FrameDrawingInfo], List[Marker]]: Visible frames and markers.
        """
        img_height, img_width = image.shape[:2]
        visible_frames = []
        visible_markers = []


        for frame in frames:
            frame_center = self.drawFrames._transformPoint(frame.transform, image_center)
            if DrawingUtils.isPointInImage(frame_center):
                visible_frames.append(frame)

        for marker in markers:
            position: Point = marker.pose.position
            px = int(image_center[0] + position.x * self.drawMarkers.conversion_ratio)
            py = int(image_center[1] - position.y * self.drawMarkers.conversion_ratio)
            
            if marker.type == Marker.ARROW:
                end_point = (px + 20, py)
                if DrawingUtils.isPointInImage((px, py)) or DrawingUtils.isPointInImage(end_point):
                    visible_markers.append(marker)

            elif marker.type == Marker.CUBE:
                side_length = int(marker.scale.x * self.drawMarkers.conversion_ratio)
                top_left = (px - side_length // 2, py - side_length // 2)
                bottom_right = (px + side_length // 2, py + side_length // 2)
                if (DrawingUtils.isPointInImage(top_left) or DrawingUtils.isPointInImage(bottom_right)):
                    visible_markers.append(marker)

            elif marker.type == Marker.SPHERE:
                radius = int(marker.scale.x * self.drawMarkers.conversion_ratio / 2)
                if DrawingUtils.isPointInImage((px - radius, py - radius)) or DrawingUtils.isPointInImage((px + radius, py + radius)):
                    visible_markers.append(marker)

        return visible_frames, visible_markers
