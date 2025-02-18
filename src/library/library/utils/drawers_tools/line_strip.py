

import numpy as np
from ...markers_management.markers import MarkerRmv
import tf_transformations as tf
from ..camera import CameraManager
import cv2

class LineStripDrawer:
    @staticmethod
    def drawLineStrip(image: np.ndarray, marker:MarkerRmv, camera_manager: CameraManager) -> None:
        if len(marker.points) < 2:
            return 
        
        quaternion = [
            marker.modified_pose.orientation.x, marker.modified_pose.orientation.y,
            marker.modified_pose.orientation.z, marker.modified_pose.orientation.w
        ]
        rotation_matrix = tf.quaternion_matrix(quaternion)[:3, :3]

        position = np.array([marker.modified_pose.position.x, marker.modified_pose.position.y, marker.modified_pose.position.z])

        transformed_points = [position + rotation_matrix @ np.array([p.x, p.y, p.z]) for p in marker.points]

        projected_points = []
        for p in transformed_points:
            camera_coords = camera_manager.worldToCamera(p)
            if camera_coords[2] > 0:  # Vérifier que le point est devant la caméra
                image_point = camera_manager.projectToImage(camera_coords)
                if image_point is not None:
                    projected_points.append(image_point)

        if len(projected_points) < 2:
            return  

        fx = camera_manager.fx
        projected_thickness = int(marker.scale.x * fx / camera_manager.camera_distance)  

        for i in range(len(projected_points) - 1):
            color = (marker.color.b * 255, marker.color.g * 255, marker.color.r * 255)

            cv2.line(image, projected_points[i], projected_points[i + 1], color, projected_thickness)
            
        