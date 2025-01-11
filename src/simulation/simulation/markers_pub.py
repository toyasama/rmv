import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
import random


class MarkersPublisher(Node):
    def __init__(self):
        super().__init__('markers_node')
        self.publisherCube = self.create_publisher(Marker, '/cube_marker', 10)
        self.publisherSphere = self.create_publisher(Marker, '/sphere_marker', 10)
        self.publisherLine = self.create_publisher(Marker, '/line_marker', 10)
        self.publisherRandom = self.create_publisher(Marker, '/random_marker', 10)

        self.cubeMarker = self._createCubeMarker()
        self.sphereMarker = self._createSphereMarker()
        self.lineMarker = self._createLineMarker()
        self.randomMarker = self._createRandomMarker()

        timerPeriod = 1.0  
        self.timer = self.create_timer(timerPeriod, self.publishMarkers)
        self.get_logger().info("MarkersPublisher initialized and running.")

    def _createCubeMarker(self):
        marker = Marker()
        marker.header.frame_id = "frame_1"
        marker.ns = "cube"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker

    def _createSphereMarker(self):
        marker = Marker()
        marker.header.frame_id = "frame_2"
        marker.ns = "sphere"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=1, nanosec=5000)
        return marker

    def _createLineMarker(self):
        marker = Marker()
        marker.header.frame_id = "frame_3"
        marker.ns = "line"
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.points = [Point(x=3.0, y=0.0, z=0.0), Point(x=3.0, y=1.0, z=0.0)]
        marker.scale.x = 0.1
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

    def _createRandomMarker(self):
        marker = Marker()
        marker.header.frame_id = "frame_4"
        marker.type = random.choice([Marker.CUBE, Marker.SPHERE, Marker.ARROW])
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def publishMarkers(self):
        self.cubeMarker.header.stamp = self.get_clock().now().to_msg()
        self.cubeMarker.pose.position.x = (self.cubeMarker.pose.position.x + 0.1 ) % 3
        self.publisherCube.publish(self.cubeMarker)

        self.sphereMarker.header.stamp = self.get_clock().now().to_msg()
        self.publisherSphere.publish(self.sphereMarker)

        self.lineMarker.header.stamp = self.get_clock().now().to_msg()
        self.publisherLine.publish(self.lineMarker)

        self.randomMarker.header.stamp = self.get_clock().now().to_msg()
        self.randomMarker.ns = f"random_ns_{random.randint(1, 100)}"
        self.randomMarker.id = random.randint(0, 1000)
        self.randomMarker.pose.position.x = random.uniform(-5.0, 5.0)
        self.randomMarker.pose.position.y = random.uniform(-5.0, 5.0)
        self.randomMarker.pose.position.z = random.uniform(-5.0, 5.0)
        self.randomMarker.scale.x = random.uniform(0.1, 1.0)
        self.randomMarker.scale.y = random.uniform(0.1, 1.0)
        self.randomMarker.scale.z = random.uniform(0.1, 1.0)
        self.randomMarker.color.r = random.uniform(0.0, 1.0)
        self.randomMarker.color.g = random.uniform(0.0, 1.0)
        self.randomMarker.color.b = random.uniform(0.0, 1.0)
        self.randomMarker.color.a = 1.0
        self.randomMarker.lifetime = Duration(sec=random.randint(1, 10))
        self.publisherRandom.publish(self.randomMarker)


def main(args=None):
    rclpy.init(args=args)
    node = MarkersPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
