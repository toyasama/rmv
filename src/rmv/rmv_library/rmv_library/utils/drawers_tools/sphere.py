
import numpy as np
from ...markers_management.markers import MarkerRmv
import tf_transformations as tf
from ..camera import CameraManager
import cv2


class SphereDrawer:
    @staticmethod
    def drawSphere(image: np.ndarray, marker:MarkerRmv, camera_manager: CameraManager) -> None:
        radius_x = marker.scale.x / 2
        radius_y = marker.scale.y / 2
        radius_z = marker.scale.z / 2

        rotation_matrix = tf.quaternion_matrix([
            marker.modified_pose.orientation.x, marker.modified_pose.orientation.y,
            marker.modified_pose.orientation.z, marker.modified_pose.orientation.w
        ])[:3, :3]

        position = np.array([marker.modified_pose.position.x, marker.modified_pose.position.y, marker.modified_pose.position.z])

        local_axes = np.array([
            [radius_x, 0, 0],  # Axe X
            [0, radius_y, 0],  # Axe Y
            [0, 0, radius_z]   # Axe Z
        ])

        rotated_axes = np.dot(rotation_matrix, local_axes.T).T


        center_camera = camera_manager.worldToCamera(position)
        if center_camera[2] <= 0:
            return 

        center_image = camera_manager.projectToImage(center_camera)

        projected_axes = []
        for axis in rotated_axes:
            world_point = position + axis
            camera_point = camera_manager.worldToCamera(world_point)
            if camera_point[2] > 0:
                projected_axes.append(camera_manager.projectToImage(camera_point))

        if len(projected_axes) == 3:
            axis_x_length = int(np.linalg.norm(np.array(projected_axes[0]) - np.array(center_image)))
            axis_y_length = int(np.linalg.norm(np.array(projected_axes[1]) - np.array(center_image)))

            color = (marker.color.b * 255, marker.color.g * 255, marker.color.r * 255)
            edge_color = (0, 0, 0)  

            cv2.ellipse(image, center_image, (axis_x_length, axis_y_length), 0, 0, 360, color, -1)
            cv2.ellipse(image, center_image, (axis_x_length, axis_y_length), 0, 0, 360, edge_color, 2)
