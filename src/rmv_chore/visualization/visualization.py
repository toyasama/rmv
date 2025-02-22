import numpy as np
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from library import( VisualizationParams,  MarkerRmv, SharedData, CameraManager, TransformDrawerInfo, TransformGraph)
from library import( DrawFrame, DrawMarkers, FramesPosition)
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
        self.params = params
        self.node = node
        self.bridge = CvBridge()
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, durability=QoSDurabilityPolicy.VOLATILE, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.draw_grid = False
        self.grid_spacing = 0.5
        self.axes_distance = 0.1
        self.image = self.createNewImage()
        self.transform_graph = transform_graph
        self.publisher_raw = self.node.create_publisher(Image, "visualization_image", qos)
        
        self.app = Flask(__name__)
        self.app.add_url_rule('/video_feed', 'video_feed', self.videoFeed)
        self.server_thread = threading.Thread(target=self.runServer, daemon=True)
        self.server_thread.start()
        self.node.get_logger().info("Visualization initialized successfully.")
        
    def drawMainFrame(self, image: np.ndarray,main_tf: str ):


        frame_pos = np.array([0, 0, 0]) 
        

        frame_pos_x_end = frame_pos + np.array([self.axes_distance, 0, 0])
        frame_pos_y_end = frame_pos + np.array([0, self.axes_distance, 0])

        proj_x_end = self.projectToImage(self.worldToCamera(frame_pos_x_end))
        proj_y_end = self.projectToImage(self.worldToCamera(frame_pos_y_end))
        proj = self.projectToImage(self.worldToCamera(frame_pos))
        frames_position = FramesPosition(main_tf,proj, proj_x_end, proj_y_end, 1)
        if proj:
            DrawFrame.drawFrame(image, frames_position)
            
    def projectAndDrawFrame(self,image :Image, transform_info: TransformDrawerInfo)->FramesPosition:
        """
        Projects a 3D point to the image plane using the intrinsic matrix.
        Args:
            point: The 3D point to project.
        Returns:
            The 2D point on the image plane.
        """
        
        quat_array = np.array([transform_info.pose_in_main_frame.rotation.w, 
                            transform_info.pose_in_main_frame.rotation.x, 
                            transform_info.pose_in_main_frame.rotation.y, 
                            transform_info.pose_in_main_frame.rotation.z])  

        # rot_mat = quat.quat2mat(quat_array)
        #TODO: check if the rotation is correct

        frame_pos = np.array([transform_info.pose_in_main_frame.translation.x, 
                            transform_info.pose_in_main_frame.translation.y, 
                            transform_info.pose_in_main_frame.translation.z]) 
        
         
        frame_pos_x_end = frame_pos + np.array([self.axes_distance, 0, 0])
        frame_pos_y_end = frame_pos + np.array([0, self.axes_distance, 0])

        proj_x_end = self.projectToImage(self.worldToCamera(frame_pos_x_end))
        proj_y_end = self.projectToImage(self.worldToCamera(frame_pos_y_end))
        proj = self.projectToImage(self.worldToCamera(frame_pos))

        start_connection =np.array([transform_info.start_connection.translation.x, transform_info.start_connection.translation.y, transform_info.start_connection.translation.z])
        end_connection =np.array([transform_info.end_connection.translation.x, transform_info.end_connection.translation.y, transform_info.end_connection.translation.z])
        
        proj_start_connection = self.projectToImage(self.worldToCamera(start_connection))
        proj_end_connection = self.projectToImage(self.worldToCamera(end_connection))
        frames_position = FramesPosition( transform_info.transform_name, proj, proj_x_end, proj_y_end,transform_info.opacity, proj_start_connection, proj_end_connection)
        if proj:
            DrawFrame.drawFrame(image, frames_position)
        else:
            print("Frame not in view")

    def generateCameraView(self,  markers: List[MarkerRmv]):
        """Generates an image centered on `main_tf` with a grid in the background."""
        
        image = self.createNewImage()
        
        main_frame = self.transform_graph.main_frame
        if not main_frame:
            return image  

        self.drawMainFrame(image, main_frame)
        tf_drawer_info: List[TransformDrawerInfo] = self.transform_graph.getTransformsFromMainFrame()

        for frame in tf_drawer_info:
            self.projectAndDrawFrame(image, frame)

        DrawMarkers.drawMarkers(image, markers, self)
        return image


# web visualization
    def isWithinBounds(self, point):
        """Checks if the projected point is within the image bounds."""
        x, y = point
        self.node.get_logger().info(f"Point: {x}, {y}")
        return 0 <= x < self.params.width and 0 <= y < self.params.height

    def visualize(self, markers: List[MarkerRmv]):
        """Génère et publie l'image en format brut et compressé."""
        self.image = self.generateCameraView( markers)

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
