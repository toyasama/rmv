
import numpy as np
from typing import List, Tuple
from ...markers_management.markers import MarkerRmv
import tf_transformations as tf
from ..camera import CameraManager
import cv2

class CubeTransformer:
    @staticmethod
    def getTransformedCorners(marker: MarkerRmv) -> np.ndarray:
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
    
    @staticmethod
    def getVisibleCorners(transformed_corners: np.ndarray, camera_manager: CameraManager) -> List[Tuple[int, int]]:
        T_camera_world = camera_manager.computeExtrinsicMatrix(np.array([0, 0, 0]))
        visible_corners = []
        for corner in transformed_corners:
            camera_coords = camera_manager.worldToCamera(corner, T_camera_world)
            if camera_coords[2] > 0:
                image_point = camera_manager.projectToImage(camera_coords)
                if image_point:
                    visible_corners.append(image_point)
        return visible_corners


class CubeDrawer:
    @staticmethod
    def drawFaces(image: np.ndarray, visible_corners: List[Tuple[int, int]], color: Tuple[int, int, int]) -> None:
        faces = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
        for face in faces:
            face_corners = [visible_corners[i] for i in face if i < len(visible_corners)]
            if len(face_corners) == 4:
                cv2.fillConvexPoly(image, np.array(face_corners, dtype=np.int32), color)
    
    @staticmethod
    def drawEdges(image: np.ndarray, visible_corners: List[Tuple[int, int]], color: Tuple[int, int, int]) -> None:
        edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
        for edge in edges:
            if edge[0] < len(visible_corners) and edge[1] < len(visible_corners):
                cv2.line(image, visible_corners[edge[0]], visible_corners[edge[1]], color, 1)
