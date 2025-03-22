
import numpy as np
from ..camera import CameraManager
import cv2

class GridDrawer:
    @staticmethod
    def drawGrid(image: np.ndarray, camera_manager:CameraManager, spacing_m: float = 1.0, color=(255, 255, 255)):
        height, width = image.shape[:2]
        thickness = max(1, int(width / camera_manager.fx))  

        max_x_m = camera_manager.camera_distance * np.tan(camera_manager.fov / 2)
        max_y_m = max_x_m * (height / width)

        x = 0
        while x < max_x_m:
            for sign in [-1, 1]:
                x_m = x * sign
                top_world = np.array([x_m, max_y_m, 0])
                bottom_world = np.array([x_m, -max_y_m, 0])

                top_img = camera_manager.projectToImage(camera_manager.worldToCamera(top_world))
                bottom_img = camera_manager.projectToImage(camera_manager.worldToCamera(bottom_world))

                if top_img and bottom_img:
                    cv2.line(image, top_img, bottom_img, color, thickness, cv2.LINE_AA) 

            x += spacing_m  

        y = 0
        while y < max_y_m:
            for sign in [-1, 1]:
                y_m = y * sign
                left_world = np.array([-max_x_m, y_m, 0])
                right_world = np.array([max_x_m, y_m, 0])

                left_img = camera_manager.projectToImage(camera_manager.worldToCamera(left_world))
                right_img = camera_manager.projectToImage(camera_manager.worldToCamera(right_world))

                if left_img and right_img:
                    cv2.line(image, left_img, right_img, color, thickness, cv2.LINE_AA)

            y += spacing_m  
            
            