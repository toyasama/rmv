import numpy as np
from .frame_position import FramesPosition
from .camera import CameraManager
import cv2
from . import drawers_tools
from typing import Tuple

class DrawFrame:
    
    @staticmethod
    def drawFrame(image: np.ndarray, frames_position: FramesPosition ):
        """
        Draws a reference frame with a small circle and optional label.
        """
        
        DrawFrame._drawAxes(image, frames_position)
        if frames_position.name:
            DrawFrame.drawText(image, frames_position.name, (frames_position.start[0] + 10, frames_position.start[1] + 10))

        if frames_position.start_connection and frames_position.end_connection:
            DrawFrame.drawArrow(image, frames_position.start_connection, frames_position.end_connection, color=(150, 150, 150), opacity=frames_position.opacity)

    
    @staticmethod
    def drawText(image: np.ndarray, text: str, position: Tuple[int, int], font_scale: float = 0.3, color: Tuple[int, int, int] = (255, 255, 255), thickness: int = 1) -> None:
        """
        Draw text on an image with optional ROI.
        """
        text_size, _ = cv2.getTextSize(text, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=font_scale, thickness=thickness)
        text_width, text_height = text_size

        x, y = position
        if x + text_width > image.shape[1]:  
            x = image.shape[1] - text_width
        if y - text_height < 0: 
            y = text_height

        
        cv2.putText(image, text, (x, y), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=font_scale, color=color, thickness=thickness, lineType=cv2.LINE_AA)
        
    @staticmethod
    def drawGrid(image: np.ndarray, spacing: int = 10, color: Tuple[int, int, int] = (255, 255, 255), thickness: int = 1):
        """
        Draws a grid on the image with the specified spacing, color, and line thickness.
        """
        height, width = image.shape[:2]
        center_x, center_y = int(width // 2), int(height // 2)
        
        for x in range(center_x, width, spacing):
            cv2.line(image, (x, 0), (x, height), color, thickness)
        for x in range(center_x, 0, -spacing):
            cv2.line(image, (x, 0), (x, height), color, thickness)

        for y in range(center_y, height, spacing):
            cv2.line(image, (0, y), (width, y), color, thickness)
        for y in range(center_y, 0, -spacing):
            cv2.line(image, (0, y), (width, y), color, thickness)
            
    
    def _drawAxes( image: np.ndarray, frame_position: FramesPosition) -> None:
        """
        Draw X and Y axes of a frame.
        """

        DrawFrame.drawArrow(image, frame_position.start, frame_position.end_x, color=(0, 0, 255))
        DrawFrame.drawArrow(image, frame_position.start, frame_position.end_y, color=(0, 255, 0))
    
    
    def drawArrow(image: np.ndarray, start: Tuple[int, int], end: Tuple[int, int], color: Tuple[int, int, int], thickness: int = 1, tip_length: float = 0.1, opacity: float = 1.0) -> None:
        """
        Draw an arrow with optional opacity and ROI.
        """
        if opacity < 1.0:
            overlay = image.copy()
            cv2.arrowedLine(overlay, start, end, color=color, thickness=thickness, tipLength=tip_length)
            cv2.addWeighted(overlay, opacity, image, 1 - opacity, 0, image)
        else:
            cv2.arrowedLine(image, start, end, color=color, thickness=thickness, tipLength=tip_length)


class DrawMarkers:
    @staticmethod
    def drawCube(image: np.ndarray, marker, camera_manager: CameraManager) -> None:
        transformed_corners = drawers_tools.CubeTransformer.getTransformedCorners(marker)
        visible_corners = drawers_tools.CubeTransformer.getVisibleCorners(transformed_corners, camera_manager)
        if len(visible_corners) >= 4:
            drawers_tools.CubeDrawer.drawFaces(image, visible_corners, (marker.color.b * 255, marker.color.g * 255, marker.color.r * 255))
            drawers_tools.CubeDrawer.drawEdges(image, visible_corners, (0, 0, 0))
    
    @staticmethod
    def drawSphere(image: np.ndarray, marker, camera_manager) -> None:
        drawers_tools.SphereDrawer.drawSphere(image, marker, camera_manager)
    
    @staticmethod
    def drawCylinder(image: np.ndarray, marker, camera_manager ) -> None:
        drawers_tools.CylinderDrawer.drawCylinder(image, marker, camera_manager)
    
    @staticmethod
    def drawArrow(image: np.ndarray, marker, camera_manager) -> None:
        drawers_tools.ArrowDrawer.drawArrow(image, marker, camera_manager)
    
    def drawLineStrip(image: np.ndarray, marker, camera_manager) -> None:
        drawers_tools.LineStripDrawer.drawLineStrip(image, marker, camera_manager)

