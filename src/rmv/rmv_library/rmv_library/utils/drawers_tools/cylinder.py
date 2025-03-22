
import numpy as np
from ...markers_management.markers import MarkerRmv
import tf_transformations as tf
from ..camera import CameraManager
import cv2


class CylinderDrawer:
    @staticmethod
    def drawCylinder(image: np.ndarray, marker:MarkerRmv, camera_manager: CameraManager) -> None:
        height = marker.scale.z
        radius_x = marker.scale.x / 2
        radius_y = marker.scale.y / 2

        rotation_matrix = tf.quaternion_matrix([
            marker.modified_pose.orientation.x, marker.modified_pose.orientation.y,
            marker.modified_pose.orientation.z, marker.modified_pose.orientation.w
        ])[:3, :3]

        position = np.array([marker.modified_pose.position.x, marker.modified_pose.position.y, marker.modified_pose.position.z])

        top_center = position + rotation_matrix @ np.array([0, 0, height / 2])
        bottom_center = position + rotation_matrix @ np.array([0, 0, -height / 2])


        top_camera = camera_manager.worldToCamera(top_center)
        bottom_camera = camera_manager.worldToCamera(bottom_center)

        if top_camera[2] > 0 and bottom_camera[2] > 0:
            top_image = camera_manager.projectToImage(top_camera)
            bottom_image = camera_manager.projectToImage(bottom_camera)

            ellipse_x = rotation_matrix @ np.array([radius_x, 0, 0])
            ellipse_y = rotation_matrix @ np.array([0, radius_y, 0])

            projected_x_top = camera_manager.worldToCamera(position + ellipse_x)
            projected_y_top = camera_manager.worldToCamera(position + ellipse_y)

            if projected_x_top[2] > 0 and projected_y_top[2] > 0:
                projected_x_top = camera_manager.projectToImage(projected_x_top)
                projected_y_top = camera_manager.projectToImage(projected_y_top)

                projected_radius_x = int(np.linalg.norm(np.array(projected_x_top) - np.array(top_image)))
                projected_radius_y = int(np.linalg.norm(np.array(projected_y_top) - np.array(top_image)))

                color = (marker.color.b * 255, marker.color.g * 255, marker.color.r * 255)
                edge_color = (0, 0, 0)

                cv2.ellipse(image, top_image, (projected_radius_x, projected_radius_y), 0, 0, 360, color, -1)
                cv2.ellipse(image, bottom_image, (projected_radius_x, projected_radius_y), 0, 0, 360, color, -1)

                cv2.ellipse(image, top_image, (projected_radius_x, projected_radius_y), 0, 0, 360, edge_color, 2)
                cv2.ellipse(image, bottom_image, (projected_radius_x, projected_radius_y), 0, 0, 360, edge_color, 2)

                cv2.line(image, (top_image[0] - projected_radius_x, top_image[1]),
                         (bottom_image[0] - projected_radius_x, bottom_image[1]), edge_color, 2)
                cv2.line(image, (top_image[0] + projected_radius_x, top_image[1]),
                         (bottom_image[0] + projected_radius_x, bottom_image[1]), edge_color, 2)

