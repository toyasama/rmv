import numpy as np
from ..markers_management.markers import MarkerRmv
import tf_transformations as tf


class CubeTransformer:
    @staticmethod
    def get_transformed_corners(marker: MarkerRmv) -> np.ndarray:
        cube_pose = marker.modified_pose
        scale = np.array([marker.scale.x , marker.scale.y , marker.scale.z ]) / 2  

        local_corners = np.array([
            [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],  # Face inférieure
            [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]       # Face supérieure
        ]) * scale  

        rotation_matrix = tf.quaternion_matrix([
            cube_pose.orientation.x, cube_pose.orientation.y,
            cube_pose.orientation.z, cube_pose.orientation.w
        ])[:3, :3]  

        rotated_corners = np.dot(local_corners, rotation_matrix.T)

        translation_vector = np.array([cube_pose.position.x, cube_pose.position.y, cube_pose.position.z])
        transformed_corners = rotated_corners + translation_vector

        return transformed_corners