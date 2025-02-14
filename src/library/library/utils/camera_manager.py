import numpy as np
from library import VisualizationParams
import tf_transformations as tf

class CameraManager:
    def __init__(self, params: VisualizationParams):
        """
        Handles all camera-related operations such as intrinsic and extrinsic matrix calculation,
        and point projection between world and camera coordinates.
        """
        self.params = params
        self.camera_distance = 8.0  
        self.fov = np.deg2rad(60)  
        self.K = self.computeIntrinsicMatrix()
    
    @property
    def fx(self):
        return self.K[0][0]

    def computeIntrinsicMatrix(self):
        """Computes the camera's intrinsic matrix."""
        fx = self.params.width / (2 * np.tan(self.fov / 2))
        fy = fx  # Square pixels
        cx = self.params.width / 2
        cy = self.params.height / 2
        return np.array([[fx, 0, cx],
                         [0, fy, cy],
                         [0, 0, 1]])

    def computeExtrinsicMatrix(self, center: np.ndarray):
        """
        Computes the extrinsic transformation matrix.
        Centers the image on `main_tf` and correctly orients the camera.
        """
        T = np.eye(4)
        T[:3, 3] = [center[0], center[1], self.camera_distance]  # Camera position
        R = tf.rotation_matrix(np.pi, [1, 0, 0])[:4, :4]
        return np.dot(T, R)

    def worldToCamera(self, point: np.ndarray, T_camera_world):
        """Converts a world point to camera coordinates."""
        world_point = np.append(point, 1)  # Homogeneous coordinates
        camera_point = np.dot(T_camera_world, world_point)
        return camera_point[:3]

    def projectToImage(self, point_camera: np.ndarray):
        """Projects a point from camera space to image space."""
        if point_camera[2] <= 0:
            return None  # Ignore if behind the camera
        
        pixel_coords = np.dot(self.K, point_camera)
        pixel_coords /= pixel_coords[2]  # Normalize
        return int(pixel_coords[0]), int(pixel_coords[1])

    def metersToPixels(self, distance_m):
        """Converts a distance in meters to pixels using the intrinsic matrix."""
        fx = self.K[0, 0]  # Get fx from the intrinsic matrix
        distance_px = distance_m * fx / self.camera_distance  # Conversion to pixels
        return int(distance_px)

class CubeTransformer:
    @staticmethod
    def get_transformed_corners(marker) -> np.ndarray:
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