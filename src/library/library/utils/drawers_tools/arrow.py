

import numpy as np
from ...markers_management.markers import MarkerRmv
import tf_transformations as tf
from ..camera import CameraManager
import cv2

class ArrowDrawer:
    @staticmethod
    def drawArrow(image: np.ndarray, marker:MarkerRmv, camera_manager:CameraManager) -> None:
        if len(marker.points) >= 2:
            start = np.array([marker.points[0].x, marker.points[0].y, marker.points[0].z])
            end = np.array([marker.points[1].x, marker.points[1].y, marker.points[1].z])
            shaft_diameter = marker.scale.x
            head_diameter = marker.scale.y
            head_length = marker.scale.z if marker.scale.z > 0 else np.linalg.norm(end - start) * 0.2
        else:
            start = np.array([marker.modified_pose.position.x, marker.modified_pose.position.y, marker.modified_pose.position.z])
            quaternion = [marker.modified_pose.orientation.x, marker.modified_pose.orientation.y, marker.modified_pose.orientation.z, marker.modified_pose.orientation.w]
            rotation_matrix = tf.quaternion_matrix(quaternion)[:3, :3]
            end = start + rotation_matrix @ np.array([marker.scale.x, 0, 0])
            shaft_diameter = marker.scale.y
            head_diameter = marker.scale.z
            head_length = marker.scale.z * 0.2

        cone_base = end - (end - start) * (head_length / np.linalg.norm(end - start))

        T_camera_world = camera_manager.computeExtrinsicMatrix(np.array([0, 0, 0]))
        start_camera = camera_manager.worldToCamera(start, T_camera_world)
        end_camera = camera_manager.worldToCamera(end, T_camera_world)
        cone_base_camera = camera_manager.worldToCamera(cone_base, T_camera_world)

        if start_camera[2] > 0 and end_camera[2] > 0 and cone_base_camera[2] > 0:
            start_image = camera_manager.projectToImage(start_camera)
            end_image = camera_manager.projectToImage(end_camera)
            cone_base_image = camera_manager.projectToImage(cone_base_camera)

            if start_image is not None and end_image is not None and cone_base_image is not None:
                fx = camera_manager.K[0, 0]
                projected_shaft_width = max(1, int(shaft_diameter * fx / start_camera[2])) 
                projected_head_width = max(2, int(head_diameter * fx / cone_base_camera[2])) 

                color = (marker.color.b * 255, marker.color.g * 255, marker.color.r * 255)
                edge_color = (0, 0, 0) 

                cv2.line(image, start_image, cone_base_image, color, projected_shaft_width)

                pts = np.array([
                    end_image,
                    (cone_base_image[0] + projected_head_width // 2, cone_base_image[1]),
                    (cone_base_image[0] - projected_head_width // 2, cone_base_image[1])
                ], dtype=np.int32)
                cv2.fillPoly(image, [pts], color)
                cv2.polylines(image, [pts], isClosed=True, color=edge_color, thickness=1)
