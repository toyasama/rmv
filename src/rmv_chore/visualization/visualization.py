import numpy as np
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from library import( RmvParameters,  MarkerRmv, CameraManager, TransformDrawerInfo, TransformGraph)
from library import( DrawFrame, DrawMarkers)
from typing import List
from sensor_msgs.msg import  Image
from flask import Flask, Response
import threading
from time import sleep

class WebServer:
    def __init__(self):
        self.image = np.zeros((100, 100, 3), dtype=np.uint8)
        self.app = Flask(__name__)
        self.app.add_url_rule('/video_feed', 'video_feed', self.videoFeed)
        self.server_thread = threading.Thread(target=self.runServer, daemon=True)
        self.server_thread.start()
        
    def updateImage(self, image: np.ndarray):
        self.image = image
        
    def runServer(self):
        self.app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)

    def generateFrames(self):
        while True:
            _, jpeg = cv2.imencode('.jpg', self.image)
            frame = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            sleep(0.03)

    def videoFeed(self):
        return Response(self.generateFrames(), mimetype='multipart/x-mixed-replace; boundary=frame')


class Visualization(CameraManager):
    def __init__(self, node: Node, params: RmvParameters, transform_graph: TransformGraph):
        """
        Initializes the visualization object, inheriting from CameraManager, with an option to draw a grid.
        """
        super().__init__(params.visualization)
        self.rmv_params = params
        self.node = node
        self.bridge = CvBridge()
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, durability=QoSDurabilityPolicy.VOLATILE, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.image = self.resetImage()
        self.transform_graph = transform_graph
        self.publisher_raw = self.node.create_publisher(Image, "visualization_image", qos)
        self.web_server = WebServer()
        self.node.get_logger().info("Visualization initialized successfully.")
        

    def updateImage(self,  markers: List[MarkerRmv]):
        """Generates an image centered on `main_tf` with a grid in the background."""
        
        self.resetImage()
        main_frame = self.transform_graph.main_frame
        if not main_frame:
            return 

        DrawFrame.drawMainFrame(self,self.image, main_frame, self.rmv_params)
        tf_drawer_info: List[TransformDrawerInfo] = self.transform_graph.getTransformsFromMainFrame()

        for frame in tf_drawer_info:
            if frame.transform_name in self.rmv_params.frames.sub_frames:   
                DrawFrame.projectAndDrawFrame(self,self.image, frame, self.rmv_params)

        DrawMarkers.drawMarkers(self.image, markers, self)

    def resetImage(self):
        """Creates a new image with a black background."""
        background_color = (self.rmv_params.visualization.background_color['b'], self.rmv_params.visualization.background_color['g'], self.rmv_params.visualization.background_color['r'])
        self.image = np.full((self.rmv_params.visualization.height, self.rmv_params.visualization.width, 3), background_color, dtype=np.uint8)
        
        if self.rmv_params.visualization.draw_grid:
            spacing_in_px = max(1 ,int((self.rmv_params.visualization.grid_spacing / self.camera_distance) * self.fx))
            grid_color = (self.rmv_params.visualization.grid_color['b'], self.rmv_params.visualization.grid_color['g'], self.rmv_params.visualization.grid_color['r'])
            DrawFrame.drawGrid(self.image,spacing_in_px, color=grid_color)

    def visualize(self, markers: List[MarkerRmv]):
        """Génère et publie l'image en format brut et compressé."""
        self.updateImage( markers)
        self.web_server.updateImage(self.image)
        ros_image = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
        self.publisher_raw.publish(ros_image)
        