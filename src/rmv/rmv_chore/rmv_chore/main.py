import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import asyncio

from rmv_chore.rmv_chore_node import RMVChoreNode
from visualization.webserber import WebServer


class Config:
    USE_WEB_SERVER = True
    USE_DESKTOP_APP = True
    PUBLISH_ON_ROS = True

RmvApp = None
if Config.USE_DESKTOP_APP:
    from visualization.app.rmv_app import RmvApp


class WebServerThread(threading.Thread):
    
    def __init__(self, node: RMVChoreNode):
        super().__init__(daemon=True)
        self.node = node
        self.web_server = WebServer()
        self.stop_event = threading.Event()

    def run(self):
        self.node.get_logger().info("Web server thread started.")
        while not self.stop_event.is_set() and self.node.visualization:
            image = self.node.visualization.image
            self.web_server.updateImage(image)
            time.sleep(self.node.period)

    def stop(self):
        self.stop_event.set()


async def start_kivy_app(node):
    if RmvApp:
        node.get_logger().info("Starting desktop app.")
        app = RmvApp(node)
        await app.async_run()  


def spin_executor(executor):
    executor.spin()


def main():
    rclpy.init()
    node = RMVChoreNode(Config.PUBLISH_ON_ROS)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    web_thread = None
    if Config.USE_WEB_SERVER:
        node.get_logger().info("Starting web server.")
        web_thread = WebServerThread(node)
        web_thread.start()

    loop = asyncio.get_event_loop()
    
    try:
        spin_thread = threading.Thread(target=spin_executor, args=(executor,), daemon=True)
        spin_thread.start()

        if Config.USE_DESKTOP_APP:
            loop.create_task(start_kivy_app(node))
        loop.run_forever()  

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if web_thread:
            web_thread.stop()
            web_thread.join()


if __name__ == "__main__":
    main()
