import numpy as np
from library import VisualizationParams
import tf_transformations as tf
from typing import Tuple, List
from ..markers_management.markers import MarkerRmv

class CameraManager:
    """
    Handles all camera-related operations such as intrinsic and extrinsic matrix calculation,
    and point projection between world and camera coordinates.
    """
    def __init__(self, params: VisualizationParams):
        self.__params = params
        self.__camera_distance = 8.0  
        self.__fov = np.deg2rad(60)  
        self.__K = self.computeIntrinsicMatrix()
        
    @property
    def camera_distance(self)-> float:
        return self.__camera_distance
        
    @property
    def params(self)-> VisualizationParams:
        return self.__params
    
    @property
    def fx(self)-> float:
        """
        Returns the focal length in the x direction.
        This is the same as the element (0, 0) of the intrinsic matrix.
        """
        return self.__K[0][0]

    def computeIntrinsicMatrix(self)-> np.ndarray:
        """Computes the camera's intrinsic matrix."""
        fx = self.__params.width / (2 * np.tan(self.__fov / 2))
        fy = fx  
        cx = self.__params.width / 2
        cy = self.__params.height / 2
        return np.array([[fx, 0, cx],
                         [0, fy, cy],
                         [0, 0, 1]])

    def computeExtrinsicMatrix(self, position: np.ndarray)-> np.ndarray:
        """
        Computes the extrinsic transformation matrix in order to move the camera to the desired position.
        args:
            position: object position in camera coordinates.
        returns:
            The extrinsic transformation matrix.
        """
        T = np.eye(4)
        T[:3, 3] = [position[0], position[1], self.__camera_distance]  
        R = tf.rotation_matrix(np.pi, [1, 0, 0])[:4, :4]
        return np.dot(T, R)

    def worldToCamera(self, point: np.ndarray, T_camera_world)-> np.ndarray:
        """Converts a world point to camera coordinates.
        args:
            point: The point to convert.
            T_camera_world: The extrinsic matrix.
        returns:
            The point in camera coordinates.
        """
        world_point = np.append(point, 1)
        camera_point = np.dot(T_camera_world, world_point)
        return camera_point[:3]

    def projectToImage(self, point_camera: np.ndarray)-> Tuple[int, int] | None:
        """Projects a point from camera space to image space."""
        if point_camera[2] <= 0:
            print("Point is behind the camera.")
            return None  
        
        pixel_coords = np.dot(self.__K, point_camera)
        pixel_coords /= pixel_coords[2] 
        return int(pixel_coords[0]), int(pixel_coords[1])
