import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf_transformations as tf
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from typing import Tuple
from library import VisualizationParams, FrameDrawingInfo, MarkerRmv, SharedData
from dataclasses import dataclass
from visualization_msgs.msg import Marker
from typing import List
from std_msgs.msg import ColorRGBA

@dataclass
class FramesPosition:
    start: Tuple[int, int]
    end_x: Tuple[int, int]
    end_y: Tuple[int, int]
    start_connection: Tuple[int, int] = None
    end_connection: Tuple[int, int] = None

class DrawingUtils:
    
    @staticmethod
    def drawFrame(image: np.ndarray, frames_position: FramesPosition,  frame_info: FrameDrawingInfo):
        """
        Draws a reference frame with a small circle and optional label.
        """
        
        DrawingUtils._drawAxes(image, frames_position)
        if frame_info.name:
            DrawingUtils.draw_text(image, frame_info.name, (frames_position.start[0] + 10, frames_position.start[1] + 10))

        if frame_info.start_connection and frame_info.end_connection:
            DrawingUtils.draw_arrow(image, frames_position.start_connection, frames_position.end_connection, color=(150, 150, 150), opacity=frame_info.opacity)

    
    @staticmethod
    def draw_text(image: np.ndarray, text: str, position: Tuple[int, int], font_scale: float = 0.3, color: Tuple[int, int, int] = (255, 255, 255), thickness: int = 1) -> None:
        """
        Draw text on an image with optional ROI.
        """
        text_size, _ = cv2.getTextSize(text, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=font_scale, thickness=thickness)
        text_width, text_height = text_size

        x, y = position
        if x + text_width > image.shape[1]:  
            x = image.shape[1] - text_width
        if y - text_height < 0: 
            y = text_height

        
        cv2.putText(image, text, (x, y), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=font_scale, color=color, thickness=thickness, lineType=cv2.LINE_AA)
        
            
    @staticmethod
    def drawGrid(image: np.ndarray, spacing: int = 10, color: Tuple[int, int, int] = (255, 255, 255), thickness: int = 1):
        """
        Draws a grid on the image with the specified spacing, color, and line thickness.
        """
        height, width = image.shape[:2]
        center_x, center_y = int(width // 2), int(height // 2)
        
        for x in range(center_x, width, spacing):
            cv2.line(image, (x, 0), (x, height), color, thickness)
        for x in range(center_x, 0, -spacing):
            cv2.line(image, (x, 0), (x, height), color, thickness)

        for y in range(center_y, height, spacing):
            cv2.line(image, (0, y), (width, y), color, thickness)
        for y in range(center_y, 0, -spacing):
            cv2.line(image, (0, y), (width, y), color, thickness)
            
    
    def _drawAxes( image: np.ndarray, frame_position: FramesPosition) -> None:
        """
        Draw X and Y axes of a frame.
        """

        DrawingUtils.draw_arrow(image, frame_position.start, frame_position.end_x, color=(0, 0, 255))
        DrawingUtils.draw_arrow(image, frame_position.start, frame_position.end_y, color=(0, 255, 0))
    
    
    def draw_arrow(image: np.ndarray, start: Tuple[int, int], end: Tuple[int, int], color: Tuple[int, int, int], thickness: int = 1, tip_length: float = 0.1, opacity: float = 1.0) -> None:
        """
        Draw an arrow with optional opacity and ROI.
        """
        if opacity < 1.0:
            overlay = image.copy()
            cv2.arrowedLine(overlay, start, end, color=color, thickness=thickness, tipLength=tip_length)
            cv2.addWeighted(overlay, opacity, image, 1 - opacity, 0, image)
        else:
            cv2.arrowedLine(image, start, end, color=color, thickness=thickness, tipLength=tip_length)

    

class CameraManager:
    def __init__(self, params: "VisualizationParams"):
        """
        Handles all camera-related operations such as intrinsic and extrinsic matrix calculation,
        and point projection between world and camera coordinates.
        """
        self.params = params
        self.camera_distance = 5.0  
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
        cube_pose = marker.pose
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

class CameraProjector:
    @staticmethod
    def get_visible_corners(transformed_corners: np.ndarray, camera_manager) -> List[Tuple[int, int]]:
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
    def draw_faces(image: np.ndarray, visible_corners: List[Tuple[int, int]], color: Tuple[int, int, int]) -> None:
        faces = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
        for face in faces:
            face_corners = [visible_corners[i] for i in face if i < len(visible_corners)]
            if len(face_corners) == 4:
                cv2.fillConvexPoly(image, np.array(face_corners, dtype=np.int32), color)
    
    @staticmethod
    def draw_edges(image: np.ndarray, visible_corners: List[Tuple[int, int]], color: Tuple[int, int, int]) -> None:
        edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
        for edge in edges:
            if edge[0] < len(visible_corners) and edge[1] < len(visible_corners):
                cv2.line(image, visible_corners[edge[0]], visible_corners[edge[1]], color, 1)


class SphereDrawer:
    @staticmethod
    def draw_sphere(image: np.ndarray, marker, camera_manager) -> None:
        radius_x = marker.scale.x / 2
        radius_y = marker.scale.y / 2
        radius_z = marker.scale.z / 2

        rotation_matrix = tf.quaternion_matrix([
            marker.pose.orientation.x, marker.pose.orientation.y,
            marker.pose.orientation.z, marker.pose.orientation.w
        ])[:3, :3]

        position = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])

        local_axes = np.array([
            [radius_x, 0, 0],  # Axe X
            [0, radius_y, 0],  # Axe Y
            [0, 0, radius_z]   # Axe Z
        ])

        rotated_axes = np.dot(rotation_matrix, local_axes.T).T

        T_camera_world = camera_manager.computeExtrinsicMatrix(np.array([0, 0, 0]))

        center_camera = camera_manager.worldToCamera(position, T_camera_world)
        if center_camera[2] <= 0:
            return 

        center_image = camera_manager.projectToImage(center_camera)

        projected_axes = []
        for axis in rotated_axes:
            world_point = position + axis
            camera_point = camera_manager.worldToCamera(world_point, T_camera_world)
            if camera_point[2] > 0:
                projected_axes.append(camera_manager.projectToImage(camera_point))

        if len(projected_axes) == 3:
            axis_x_length = int(np.linalg.norm(np.array(projected_axes[0]) - np.array(center_image)))
            axis_y_length = int(np.linalg.norm(np.array(projected_axes[1]) - np.array(center_image)))

            color = (marker.color.b * 255, marker.color.g * 255, marker.color.r * 255)
            edge_color = (0, 0, 0)  # Noir pour le contour

            cv2.ellipse(image, center_image, (axis_x_length, axis_y_length), 0, 0, 360, color, -1)
            cv2.ellipse(image, center_image, (axis_x_length, axis_y_length), 0, 0, 360, edge_color, 2)


class CylinderDrawer:
    @staticmethod
    def draw_cylinder(image: np.ndarray, marker, camera_manager) -> None:
        height = marker.scale.z
        radius_x = marker.scale.x / 2
        radius_y = marker.scale.y / 2

        rotation_matrix = tf.quaternion_matrix([
            marker.pose.orientation.x, marker.pose.orientation.y,
            marker.pose.orientation.z, marker.pose.orientation.w
        ])[:3, :3]

        position = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])

        top_center = position + rotation_matrix @ np.array([0, 0, height / 2])
        bottom_center = position + rotation_matrix @ np.array([0, 0, -height / 2])

        T_camera_world = camera_manager.computeExtrinsicMatrix(np.array([0, 0, 0]))

        top_camera = camera_manager.worldToCamera(top_center, T_camera_world)
        bottom_camera = camera_manager.worldToCamera(bottom_center, T_camera_world)

        if top_camera[2] > 0 and bottom_camera[2] > 0:  # Vérifier si dans le champ de vision
            top_image = camera_manager.projectToImage(top_camera)
            bottom_image = camera_manager.projectToImage(bottom_camera)

            ellipse_x = rotation_matrix @ np.array([radius_x, 0, 0])
            ellipse_y = rotation_matrix @ np.array([0, radius_y, 0])

            projected_x_top = camera_manager.worldToCamera(position + ellipse_x, T_camera_world)
            projected_y_top = camera_manager.worldToCamera(position + ellipse_y, T_camera_world)

            if projected_x_top[2] > 0 and projected_y_top[2] > 0:
                projected_x_top = camera_manager.projectToImage(projected_x_top)
                projected_y_top = camera_manager.projectToImage(projected_y_top)

                projected_radius_x = int(np.linalg.norm(np.array(projected_x_top) - np.array(top_image)))
                projected_radius_y = int(np.linalg.norm(np.array(projected_y_top) - np.array(top_image)))

                # Couleurs
                color = (marker.color.b * 255, marker.color.g * 255, marker.color.r * 255)
                edge_color = (0, 0, 0)  # Contours noirs pour effet 3D

                cv2.ellipse(image, top_image, (projected_radius_x, projected_radius_y), 0, 0, 360, color, -1)
                cv2.ellipse(image, bottom_image, (projected_radius_x, projected_radius_y), 0, 0, 360, color, -1)

                cv2.ellipse(image, top_image, (projected_radius_x, projected_radius_y), 0, 0, 360, edge_color, 2)
                cv2.ellipse(image, bottom_image, (projected_radius_x, projected_radius_y), 0, 0, 360, edge_color, 2)

                cv2.line(image, (top_image[0] - projected_radius_x, top_image[1]),
                         (bottom_image[0] - projected_radius_x, bottom_image[1]), edge_color, 2)
                cv2.line(image, (top_image[0] + projected_radius_x, top_image[1]),
                         (bottom_image[0] + projected_radius_x, bottom_image[1]), edge_color, 2)


class ArrowDrawer:
    @staticmethod
    def draw_arrow(image: np.ndarray, marker, camera_manager) -> None:
        if len(marker.points) >= 2:
            # Mode Start/End
            start = np.array([marker.points[0].x, marker.points[0].y, marker.points[0].z])
            end = np.array([marker.points[1].x, marker.points[1].y, marker.points[1].z])
            shaft_diameter = marker.scale.x
            head_diameter = marker.scale.y
            head_length = marker.scale.z if marker.scale.z > 0 else np.linalg.norm(end - start) * 0.2
        else:
            # Mode Position/Orientation
            start = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
            quaternion = [marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w]
            rotation_matrix = tf.quaternion_matrix(quaternion)[:3, :3]
            end = start + rotation_matrix @ np.array([marker.scale.x, 0, 0])
            shaft_diameter = marker.scale.y
            head_diameter = marker.scale.z
            head_length = marker.scale.z * 0.2

        # Calcul de la base du cône
        cone_base = end - (end - start) * (head_length / np.linalg.norm(end - start))

        # Transformation vers la caméra
        T_camera_world = camera_manager.computeExtrinsicMatrix(np.array([0, 0, 0]))
        start_camera = camera_manager.worldToCamera(start, T_camera_world)
        end_camera = camera_manager.worldToCamera(end, T_camera_world)
        cone_base_camera = camera_manager.worldToCamera(cone_base, T_camera_world)

        if start_camera[2] > 0 and end_camera[2] > 0 and cone_base_camera[2] > 0:
            start_image = camera_manager.projectToImage(start_camera)
            end_image = camera_manager.projectToImage(end_camera)
            cone_base_image = camera_manager.projectToImage(cone_base_camera)

            if start_image is not None and end_image is not None and cone_base_image is not None:
                # Adapter la taille projetée avec la perspective
                fx = camera_manager.K[0, 0]
                projected_shaft_width = max(1, int(shaft_diameter * fx / start_camera[2]))  # Évite d'avoir 0 px
                projected_head_width = max(2, int(head_diameter * fx / cone_base_camera[2]))  # Doit être plus large que le shaft

                # Couleurs
                color = (marker.color.b * 255, marker.color.g * 255, marker.color.r * 255)
                edge_color = (0, 0, 0)  # Contours noirs pour le contraste

                # Dessiner la ligne centrale
                cv2.line(image, start_image, cone_base_image, color, projected_shaft_width)

                # Dessiner la pointe conique avec adaptation perspective
                pts = np.array([
                    end_image,
                    (cone_base_image[0] + projected_head_width // 2, cone_base_image[1]),
                    (cone_base_image[0] - projected_head_width // 2, cone_base_image[1])
                ], dtype=np.int32)
                cv2.fillPoly(image, [pts], color)
                cv2.polylines(image, [pts], isClosed=True, color=edge_color, thickness=1)



class LineStripDrawer:
    @staticmethod
    def draw_line_strip(image: np.ndarray, marker:MarkerRmv, camera_manager: CameraManager) -> None:
        if len(marker.points) < 2:
            return  # Pas assez de points pour former une ligne

        # Extraire la matrice de rotation à partir du quaternion de la pose
        quaternion = [
            marker.pose.orientation.x, marker.pose.orientation.y,
            marker.pose.orientation.z, marker.pose.orientation.w
        ]
        rotation_matrix = tf.quaternion_matrix(quaternion)[:3, :3]

        # Extraire la position de la pose
        position = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])

        # Transformer tous les points en tenant compte de la pose
        transformed_points = [position + rotation_matrix @ np.array([p.x, p.y, p.z]) for p in marker.points]

        # Calculer la transformation vers la caméra
        T_camera_world = camera_manager.computeExtrinsicMatrix(np.array([0, 0, 0]))
        projected_points = []
        for p in transformed_points:
            camera_coords = camera_manager.worldToCamera(p, T_camera_world)
            if camera_coords[2] > 0:  # Vérifier que le point est devant la caméra
                image_point = camera_manager.projectToImage(camera_coords)
                if image_point is not None:
                    projected_points.append(image_point)

        if len(projected_points) < 2:
            return  # Pas assez de points projetés pour former une ligne

        fx = camera_manager.fx
        projected_thickness = int(marker.scale.x * fx / camera_manager.camera_distance)  # Ajuster l'épaisseur

        # Dessiner les segments de la ligne
        for i in range(len(projected_points) - 1):
            color = (marker.color.b * 255, marker.color.g * 255, marker.color.r * 255)

            cv2.line(image, projected_points[i], projected_points[i + 1], color, projected_thickness)

class DrawMarkers:
    @staticmethod
    def drawCube(image: np.ndarray, marker, camera_manager) -> None:
        transformed_corners = CubeTransformer.get_transformed_corners(marker)
        visible_corners = CameraProjector.get_visible_corners(transformed_corners, camera_manager)
        if len(visible_corners) >= 4:
            CubeDrawer.draw_faces(image, visible_corners, (marker.color.b * 255, marker.color.g * 255, marker.color.r * 255))
            CubeDrawer.draw_edges(image, visible_corners, (0, 0, 0))
    
    @staticmethod
    def drawSphere(image: np.ndarray, marker, camera_manager) -> None:
        SphereDrawer.draw_sphere(image, marker, camera_manager)
    
    @staticmethod
    def drawCylinder(image: np.ndarray, marker, camera_manager ) -> None:
        CylinderDrawer.draw_cylinder(image, marker, camera_manager)
    
    @staticmethod
    def drawArrow(image: np.ndarray, marker, camera_manager) -> None:
        ArrowDrawer.draw_arrow(image, marker, camera_manager)
    
    def drawLineStrip(image: np.ndarray, marker, camera_manager) -> None:
        LineStripDrawer.draw_line_strip(image, marker, camera_manager)

class GridDrawer:
    @staticmethod
    def drawGrid(image: np.ndarray, camera_manager, spacing_m: float = 1.0, color=(255, 255, 255)):
        height, width = image.shape[:2]
        thickness = max(1, int(width / camera_manager.fx))  # Ajuster selon la taille

        T_camera_world = camera_manager.computeExtrinsicMatrix(np.array([0, 0, 0]))
        max_x_m = camera_manager.camera_distance * np.tan(camera_manager.fov / 2)
        max_y_m = max_x_m * (height / width)

        x = 0
        while x < max_x_m:
            for sign in [-1, 1]:
                x_m = x * sign
                top_world = np.array([x_m, max_y_m, 0])
                bottom_world = np.array([x_m, -max_y_m, 0])

                top_img = camera_manager.projectToImage(camera_manager.worldToCamera(top_world, T_camera_world))
                bottom_img = camera_manager.projectToImage(camera_manager.worldToCamera(bottom_world, T_camera_world))

                if top_img and bottom_img:
                    cv2.line(image, top_img, bottom_img, color, thickness, cv2.LINE_AA)  # Anti-aliasing

            x += spacing_m  

        y = 0
        while y < max_y_m:
            for sign in [-1, 1]:
                y_m = y * sign
                left_world = np.array([-max_x_m, y_m, 0])
                right_world = np.array([max_x_m, y_m, 0])

                left_img = camera_manager.projectToImage(camera_manager.worldToCamera(left_world, T_camera_world))
                right_img = camera_manager.projectToImage(camera_manager.worldToCamera(right_world, T_camera_world))

                if left_img and right_img:
                    cv2.line(image, left_img, right_img, color, thickness, cv2.LINE_AA)

            y += spacing_m  


class Visualization(CameraManager):
    def __init__(self, node: "Node", params: "VisualizationParams", shared_data: "SharedData"):
        """
        Initializes the visualization object, inheriting from CameraManager, with an option to draw a grid.
        """
        super().__init__(params)
        self.node = node
        self.shared_data = shared_data
        self.bridge = CvBridge()
        self.publisher = self.node.create_publisher(Image, "visualization_image", qos_profile_sensor_data)
        self.draw_grid = True
        self.grid_spacing = 0.5
        self.axes_distance = 0.1
        self.image = self.createNewImage()

    def generateCameraView(self):
        """Generates an image centered on `main_tf` with a grid in the background."""
        image = self.createNewImage()
        main_tf = self.shared_data.get_main_tf()
        if not main_tf.name:
            return image  # No reference point found

        center_position = np.array([main_tf.transform.translation.x,
                                    main_tf.transform.translation.y,
                                    main_tf.transform.translation.z])

        # Update extrinsic matrix centered on `main_tf`
        T_camera_world = self.computeExtrinsicMatrix(center_position)


        frame_pos_x_end = center_position + np.array([self.axes_distance, 0, 0])
        frame_pos_y_end = center_position + np.array([0,self.axes_distance, 0])
        proj_x_end = self.projectToImage(self.worldToCamera(frame_pos_x_end, T_camera_world))
        proj_y_end = self.projectToImage(self.worldToCamera(frame_pos_y_end, T_camera_world))
        proj_main = self.projectToImage(self.worldToCamera(center_position, T_camera_world))
        frames_position = FramesPosition(proj_main, proj_x_end, proj_y_end)
        if proj_main:
            DrawingUtils.drawFrame(image, frames_position, main_tf)

        for frame in self.shared_data.get_other_tfs():
            frame_pos = np.array([frame.transform.translation.x, frame.transform.translation.y, frame.transform.translation.z])
            frame_pos_x_end = frame_pos + np.array([self.axes_distance, 0, 0])
            frame_pos_y_end = frame_pos + np.array([0,self.axes_distance, 0])
            proj_x_end = self.projectToImage(self.worldToCamera(frame_pos_x_end, T_camera_world))
            proj_y_end = self.projectToImage(self.worldToCamera(frame_pos_y_end, T_camera_world))
            proj = self.projectToImage(self.worldToCamera(frame_pos, T_camera_world))
            
            start_connection =np.array([frame.start_connection.translation.x, frame.start_connection.translation.y, frame.start_connection.translation.z])
            end_connection =np.array([frame.end_connection.translation.x, frame.end_connection.translation.y, frame.end_connection.translation.z])
            
            proj_start_connection = self.projectToImage(self.worldToCamera(start_connection, T_camera_world))
            proj_end_connection = self.projectToImage(self.worldToCamera(end_connection, T_camera_world))
            
            frames_position = FramesPosition(proj, proj_x_end, proj_y_end, proj_start_connection, proj_end_connection)
            if proj:
                DrawingUtils.drawFrame(image, frames_position, frame)

        for marker in self.shared_data.get_markers():
            match marker.type:
                case  Marker.CUBE:
                    DrawMarkers.drawCube(image, marker, self)
                    
                case  Marker.SPHERE:
                    DrawMarkers.drawSphere(image, marker, self)
                
                case  Marker.CYLINDER:
                    DrawMarkers.drawCylinder(image, marker, self)
                    
                case  Marker.LINE_STRIP:
                    DrawMarkers.drawLineStrip(image, marker, self)
        return image

    def isWithinBounds(self, point):
        """Checks if the projected point is within the image bounds."""
        x, y = point
        self.node.get_logger().info(f"Point: {x}, {y}")
        return 0 <= x < self.params.width and 0 <= y < self.params.height

    def run(self):
        """Updates and publishes the image."""
        self.image = self.generateCameraView()
        ros_image = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
        self.publisher.publish(ros_image)

    def createNewImage(self):
        """Creates a new image with a black background."""
        image = np.full((self.params.height, self.params.width, 3), (0, 0, 0), dtype=np.uint8)
        
        if self.draw_grid:
            GridDrawer.drawGrid(image, self,self.axes_distance)
        return image
