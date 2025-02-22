from dataclasses import dataclass
import numpy as np
import tf_transformations as tf
from typing import Tuple
from ..parameters.params import VisualizationParams

class CameraExtrinsics:
    """Extrinsic parameters of the camera."""
    def __init__(self, x: float, y: float, z: float, theta: float):
        self.__x: float = x
        self.__y: float = y
        self.__z: float = z
        self.__theta: float = theta
    
    @property
    def extrinsic_matrix(self) -> np.ndarray:
        """
        Computes the extrinsic transformation matrix to move the camera to the desired position.
        Returns:
            The extrinsic transformation matrix as a NumPy array.
        """
        T = np.eye(4)
        T[:3, 3] = np.array([self.__x, self.__y, self.__z])
        T[:3, :3] = tf.rotation_matrix(np.pi, [1, 0, 0])[:3, :3] @ tf.rotation_matrix(self.__theta, [0, 0, 1])[:3, :3]
        return T

class CameraIntrinsics:
    """Intrinsic parameters of the camera."""
    def __init__(self, width: int, height: int, fov: float):
        self.__width: int = width
        self.__height: int = height
        self.__fov: float = fov

    @property
    def intrinsic_matrix(self) -> np.ndarray:
        """Computes the camera's intrinsic matrix."""
        fx = self.__width / (2 * np.tan(self.__fov / 2))
        fy = fx
        cx = self.__width / 2
        cy = self.__height / 2
        return np.array([[fx, 0, cx],
                         [0, fy, cy],
                         [0, 0, 1]])

class CameraManager:
    """
    Handles all camera-related operations such as intrinsic and extrinsic matrix calculation,
    and point projection between world and camera coordinates.
    """
    def __init__(self, params: VisualizationParams):
        self.__intrinsics = CameraIntrinsics(params.width, params.height, np.deg2rad(60))
        self.__extrinsics = CameraExtrinsics(0, 0, 5, 0)
    
    def worldToCamera(self, point: np.ndarray) -> np.ndarray:
        """Converts a world point to camera coordinates."""
        T_camera_world = self.__extrinsics.extrinsic_matrix
        world_point = np.append(point, 1)
        camera_point = np.dot(T_camera_world, world_point)
        return camera_point[:3]

    def projectToImage(self, point_camera: np.ndarray) -> Tuple[int, int] | None:
        """Projects a point from camera space to image space."""
        if point_camera[2] <= 0:
            print("Point is behind the camera.")
            return None  
        pixel_coords = np.dot(self.__intrinsics.intrinsic_matrix, point_camera)
        pixel_coords /= pixel_coords[2] 
        return int(pixel_coords[0]), int(pixel_coords[1])
    
    @property
    def fx(self) -> float:
        """Gets the focal length in the x-direction."""
        return self.__intrinsics.intrinsic_matrix[0, 0]
    
    @property
    def camera_distance(self) -> float:
        """Gets the distance between the camera and the origin."""
        return self.__extrinsics.extrinsic_matrix[2, 3]