import numpy as np
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from library import( VisualizationParams,  MarkerRmv, SharedData, CameraManager, TransformDrawerInfo, TransformGraph)
from library import( DrawFrame, DrawMarkers, FramesPosition)
from visualization_msgs.msg import Marker
from typing import List
from sensor_msgs.msg import  Image
from flask import Flask, Response
import threading
from time import sleep



class Visualization(CameraManager):
    pass
    def __init__(self, node: "Node", params: "VisualizationParams", transform_graph: TransformGraph):
        """
        Initializes the visualization object, inheriting from CameraManager, with an option to draw a grid.
        """
        super().__init__(params)
        self.node = node
        self.bridge = CvBridge()
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, durability=QoSDurabilityPolicy.VOLATILE, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.draw_grid = False
        self.grid_spacing = 0.5
        self.axes_distance = 0.1
        self.image = self.createNewImage()
        self.shared_data = SharedData()
        self.transform_graph = transform_graph
        self.publisher_raw = self.node.create_publisher(Image, "visualization_image", qos)
        
        self.app = Flask(__name__)
        self.app.add_url_rule('/video_feed', 'video_feed', self.videoFeed)
        self.server_thread = threading.Thread(target=self.runServer, daemon=True)
        self.server_thread.start()
        self.node.get_logger().info("Visualization initialized successfully.")

    def generateCameraView(self, tf_drawer_info: List[TransformDrawerInfo], markers: List[MarkerRmv]):
        """Generates an image centered on `main_tf` with a grid in the background."""
        image = self.createNewImage()
        if not tf_drawer_info:
            return image  # No reference point found
        main_tf = "frame_1"
        if not main_tf:
            return image  # No reference point found

        # center_position = np.array([main_tf.transform.translation.x,
        #                             main_tf.transform.translation.y,
        #                             main_tf.transform.translation.z])

        center_position = np.array([0,0,0])
        # Update extrinsic matrix centered on `main_tf`
        T_camera_world = self.computeExtrinsicMatrix(center_position)


        frame_pos_x_end = center_position + np.array([self.axes_distance, 0, 0])
        frame_pos_y_end = center_position + np.array([0,self.axes_distance, 0])
        proj_x_end = self.projectToImage(self.worldToCamera(frame_pos_x_end, T_camera_world))
        proj_y_end = self.projectToImage(self.worldToCamera(frame_pos_y_end, T_camera_world))
        proj_main = self.projectToImage(self.worldToCamera(center_position, T_camera_world))
        # frames_position = FramesPosition(proj_main, proj_x_end, proj_y_end)
        # if proj_main:
        #     DrawFrame.drawFrame(image, frames_position, main_tf)

        for frame in tf_drawer_info:
            frame_pos = np.array([frame.pose_in_main_frame.translation.x, frame.pose_in_main_frame.translation.y, frame.pose_in_main_frame.translation.z])
            frame_pos_x_end = frame_pos + np.array([self.axes_distance, 0, 0])
            frame_pos_y_end = frame_pos + np.array([0,self.axes_distance, 0])
            proj_x_end = self.projectToImage(self.worldToCamera(frame_pos_x_end, T_camera_world))
            proj_y_end = self.projectToImage(self.worldToCamera(frame_pos_y_end, T_camera_world))
            proj = self.projectToImage(self.worldToCamera(frame_pos, T_camera_world))
            
            start_connection =np.array([frame.start_connection.translation.x, frame.start_connection.translation.y, frame.start_connection.translation.z])
            end_connection =np.array([frame.end_connection.translation.x, frame.end_connection.translation.y, frame.end_connection.translation.z])
            
            proj_start_connection = self.projectToImage(self.worldToCamera(start_connection, T_camera_world))
            proj_end_connection = self.projectToImage(self.worldToCamera(end_connection, T_camera_world))
            
            frames_position = FramesPosition( frame.transform_name, proj, proj_x_end, proj_y_end,frame.opacity, proj_start_connection, proj_end_connection)
            if proj:
                DrawFrame.drawFrame(image, frames_position)
            else:
                print("Frame not in view")

        for marker in markers:
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


# web visualization
    def isWithinBounds(self, point):
        """Checks if the projected point is within the image bounds."""
        x, y = point
        self.node.get_logger().info(f"Point: {x}, {y}")
        return 0 <= x < self.params.width and 0 <= y < self.params.height

    def visualize(self, markers: List[MarkerRmv]):
        """Génère et publie l'image en format brut et compressé."""
        tf_drawer_info = self.transform_graph.getTransformsFromMainFrame()
        self.image = self.generateCameraView(tf_drawer_info, markers)

        ros_image = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
        self.publisher_raw.publish(ros_image)
        
    def createNewImage(self):
        """Creates a new image with a black background."""
        image = np.full((self.params.height, self.params.width, 3), (0, 0, 0), dtype=np.uint8)
        
        if self.draw_grid:
            DrawFrame.drawGrid(image, self,self.axes_distance)
        return image

    def runServer(self):
        """Lance le serveur Flask."""
        self.app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)

    def generateFrames(self):
        """Génère les images pour le flux vidéo."""
        while True:
            _, jpeg = cv2.imencode('.jpg', self.image)
            frame = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            sleep(0.03)

    def videoFeed(self):
        """Route Flask pour le flux vidéo."""
        return Response(self.generateFrames(), mimetype='multipart/x-mixed-replace; boundary=frame')
