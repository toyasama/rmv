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
        self._image = np.zeros((100, 100, 3), dtype=np.uint8)
        self.app = Flask(__name__)
        self.app.add_url_rule('/video_feed', 'video_feed', self.videoFeed)
        self.server_thread = threading.Thread(target=self.runServer, daemon=True)
        self.server_thread.start()
        
    def updateImage(self, image: np.ndarray):
        self._image = image
        
    def runServer(self):
        self.app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)

    def generateFrames(self):
        while True:
            _, jpeg = cv2.imencode('.jpg', self._image)
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
        self.image_lock = threading.RLock()
        self.rmv_params = params
        self.node = node
        self.bridge = CvBridge()
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, durability=QoSDurabilityPolicy.VOLATILE, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._image = self.resetImage()
        self.transform_graph = transform_graph
        self.publisher_raw = self.node.create_publisher(Image, "visualization_image", qos)
        self.web_server = WebServer()
        self.node.get_logger().info("Visualization initialized successfully.")
        
    @property
    def image(self):
        with self.image_lock:
            if self._image is None:
                return None
            return self._image.copy()

    def updateImage(self,  markers: List[MarkerRmv]):
        """Generates an image centered on `main_tf` with a grid in the background."""
        self.resetImage()
        main_frame = self.transform_graph.main_frame
        if not main_frame:
            return 
        axe_length_px = int(self.rmv_params.frames.axes_length * 0.01 * self.fx / self.camera_distance)
        axes_thickness = max(1, axe_length_px)
        with self.image_lock:
            DrawFrame.drawMainFrame(self,self._image, main_frame, self.rmv_params, axes_thickness)
        tf_drawer_info: List[TransformDrawerInfo] = self.transform_graph.getTransformsFromMainFrame()

        for frame in tf_drawer_info:
            if self.rmv_params.frames.show_sub_frames:
                with self.image_lock:  
                    DrawFrame.projectAndDrawFrame(self,self._image, frame, self.rmv_params, axes_thickness)
            elif frame.transform_name in self.rmv_params.frames.sub_frames: 
                with self.image_lock:
                    DrawFrame.projectAndDrawFrame(self,self._image, frame, self.rmv_params, axes_thickness)
        with self.image_lock:
            DrawMarkers.drawMarkers(self._image, markers, self)

    def resetImage(self):
        """Creates a new image with a black background."""
        background_color = (self.rmv_params.visualization.background_color['b'], self.rmv_params.visualization.background_color['g'], self.rmv_params.visualization.background_color['r'])
        with self.image_lock:
            self._image = np.full((self.rmv_params.visualization.height, self.rmv_params.visualization.width, 3), background_color, dtype=np.uint8)
        
        if self.rmv_params.visualization.draw_grid:
            spacing_in_px = max(1 ,int((self.rmv_params.visualization.grid_spacing / self.camera_distance) * self.fx))
            grid_color = (self.rmv_params.visualization.grid_color['b'], self.rmv_params.visualization.grid_color['g'], self.rmv_params.visualization.grid_color['r'])
            with self.image_lock:
                grid_thickness = max(1, int(0.01*spacing_in_px))
                DrawFrame.drawGrid(self._image,spacing_in_px, color=grid_color, thickness=grid_thickness)

    def visualize(self, markers: List[MarkerRmv]):
        """Génère et publie l'image en format brut et compressé."""
        self.updateImage( markers)
        with self.image_lock:
            self.web_server.updateImage(self._image)
            ros_image = self.bridge.cv2_to_imgmsg(self._image, encoding="bgr8")
        self.publisher_raw.publish(ros_image)
        