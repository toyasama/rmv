import threading
import numpy as np
import cv2
from flask import Flask, Response
from time import sleep

class WebServer:
    def __init__(self):
        self._image = np.zeros((100, 100, 3), dtype=np.uint8)
        self.app = Flask(__name__)
        self.app.add_url_rule('/rmv/stream', 'rmv/stream', self.videoFeed)
        self.app.add_url_rule('/', 'index', self.index)
        self.server_thread = threading.Thread(target=self.runServer, daemon=True)
        self.server_thread.start()
        print("Web server started.")
        
    def updateImage(self, image: np.ndarray):
        self._image = image
        
    def runServer(self):
        self.app.run(host="localhost", port=5000, debug=False, threaded=True)

    def generateFrames(self):
        while True:
            _, jpeg = cv2.imencode('.jpg', self._image)
            frame = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            sleep(0.03)

    def videoFeed(self):
        return Response(self.generateFrames(), mimetype='multipart/x-mixed-replace; boundary=frame')
    
    def index(self):
        """Page d'accueil avec un peu de style et une interface plus jolie"""
        html_content = """
        <html>
        <head>
            <title>RMV Stream</title>
            <style>
                body {
                    font-family: Arial, sans-serif;
                    margin: 0;
                    padding: 0;
                    background-color: #f0f0f0;
                    display: flex;
                    justify-content: center;
                    align-items: center;
                    height: 100vh;
                }
                .container {
                    text-align: center;
                    background-color: #fff;
                    padding: 20px;
                    border-radius: 10px;
                    box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
                    width: 80%;
                    max-width: 800px;
                }
                h1 {
                    color: #333;
                    font-size: 36px;
                }
                p {
                    font-size: 18px;
                    color: #666;
                }
                .video {
                    margin-top: 20px;
                    max-width: 100%;
                    border-radius: 10px;
                    overflow: hidden;
                    box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
                }
                .footer {
                    margin-top: 30px;
                    font-size: 14px;
                    color: #aaa;
                }
            </style>
        </head>
        <body>
            <div class="container">
                <h1>RMV Video Stream</h1>
                <p>Live streaming of the RMV visualization.</p>
                <div class="video">
                    <img src="/rmv/stream" width="100%" />
                </div>
            </div>
        </body>
        </html>
        """
        return html_content
