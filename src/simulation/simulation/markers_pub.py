import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
import random
import math

class MarkersPublisher(Node):
    def __init__(self):
        super().__init__('markers_node')
        self.publisherCube = self.create_publisher(Marker, '/cube_marker', 10)
        self.publisherSphere = self.create_publisher(Marker, '/sphere_marker', 10)
        self.publisherCylinder = self.create_publisher(Marker, '/cylinder_marker', 10)
        self.publisherArrow = self.create_publisher(Marker, '/arrow_marker', 10)
        self.publisherRandom = self.create_publisher(Marker, '/random_marker', 10)
        self.publisherCircle = self.create_publisher(Marker, '/circle_marker', 10)
        self.publisherLine = self.create_publisher(Marker, '/line_marker', 10)  

        self.cubeMarker = self._createCubeMarker()
        self.sphereMarker = self._createSphereMarker()
        self.arrowMarker = self._createArrowMarker()
        self.randomMarker = self._createRandomMarker()
        self.cylinderMarker = self._createCylinderMarker()
        self.circleMarker = self._createCircleMarker()
        self.lineMarker = self._createLineMarker()  

        timerPeriod = 0.1  
        self.timer = self.create_timer(timerPeriod, self.publishMarkers)
        self.get_logger().info("MarkersPublisher initialized and running.")
    
    def _createCylinderMarker(self):
        marker = Marker()
        marker.header.frame_id = "frame_1"
        marker.ns = "cylinder"
        marker.id = 30
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = -1.5
        marker.pose.position.y = 2.3
        angle_rad = math.radians(45)  # Conversion de 30° en radians
        marker.pose.orientation.x = math.sin(angle_rad / 2)
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = math.cos(angle_rad / 2)
        marker.scale.x = 0.25
        marker.scale.y = 0.1
        marker.scale.z = 0.5
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=1, nanosec=5000)
        return marker
        

    def _createCubeMarker(self):
        marker = Marker()
        marker.header.frame_id = "frame_1"
        marker.ns = "cube"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 2.0
        marker.pose.position.y = -1.0
        angle_rad = math.radians(50)  # Conversion de 30° en radians
        marker.pose.orientation.x = math.sin(angle_rad / 2)
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = math.cos(angle_rad / 2)
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=1, nanosec=5000)
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
        angle_rad = math.radians(60)  # Conversion de 30° en radians
        marker.pose.orientation.x = math.sin(angle_rad / 2)
        marker.pose.orientation.y = 1.5
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = math.cos(angle_rad / 2)
        marker.lifetime = Duration(sec=1, nanosec=5000)
        return marker
    
    
    def _createCircleMarker(self):
        marker = Marker()
        marker.header.frame_id = "frame_1"
        marker.ns = "Circle"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.0
        marker.color.r = 0.0
        marker.color.g = 0.7
        marker.color.b = 0.8
        marker.color.a = 1.0
        marker.pose.position.x = -2.0
        marker.pose.position.y = 1.0
        angle_rad = math.radians(80)  # Conversion de 30° en radians
        marker.pose.orientation.x = math.sin(angle_rad / 2)
        marker.pose.orientation.y = 1.5
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = math.cos(angle_rad / 2)
        marker.lifetime = Duration(sec=1, nanosec=5000)
        return marker

    def _createArrowMarker(self):
        marker = Marker()
        marker.header.frame_id = "frame_1"
        marker.ns = "arrow"
        marker.id = 2
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        # marker.points = [Point(x=-1.0, y=-2.0, z=0.0), Point(x=1.0, y=1.0, z=0.0)]
        marker.pose.position.x = 2.0
        marker.pose.position.y = 1.0
        angle_rad = math.radians(180)  # Conversion de 30° en radians
        marker.pose.orientation.x = math.sin(angle_rad / 2)
        marker.pose.orientation.y = 1.5
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = math.cos(angle_rad / 2)
        marker.scale.x = 0.6
        marker.scale.y = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=1, nanosec=5000)
        return marker

    def _createRandomMarker(self):
        marker = Marker()
        marker.header.frame_id = "frame_4"
        marker.type = random.choice([Marker.CUBE, Marker.SPHERE, Marker.ARROW])
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    # Ajoutez cette méthode pour créer le marqueur de ligne
    def _createLineMarker(self):
        marker = Marker()
        marker.header.frame_id = "frame_1"
        marker.ns = "line"
        marker.id = 40
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=0.7, y=0.5, z=0.0), Point(x=1.0, y=0.3, z=0.0), Point(x=1.5, y=0.5, z=0.0)]
        marker.lifetime = Duration(sec=1, nanosec=5000)
        return marker

    def publishMarkers(self):
        self.cubeMarker.header.stamp = self.get_clock().now().to_msg()
        self.cubeMarker.pose.position.x = (self.cubeMarker.pose.position.x + 0.05 ) % 3
        self.cubeMarker.pose.position.y = (self.cubeMarker.pose.position.y - 0.05 ) % 3
        self.publisherCube.publish(self.cubeMarker)

        self.sphereMarker.header.stamp = self.get_clock().now().to_msg()
        self.sphereMarker.pose.position.x = (self.sphereMarker.pose.position.x + 0.05 ) % 3
        self.sphereMarker.pose.position.y = (self.sphereMarker.pose.position.y + 0.05 ) % 3
        self.publisherSphere.publish(self.sphereMarker)
        
        self.cylinderMarker.header.stamp = self.get_clock().now().to_msg()
        self.cylinderMarker.pose.position.x = (self.cylinderMarker.pose.position.x + 0.05 ) % 3
        self.cylinderMarker.pose.position.y = (self.cylinderMarker.pose.position.y - 0.05 ) % 3
        self.publisherCylinder.publish(self.cylinderMarker)
        

        self.arrowMarker.header.stamp = self.get_clock().now().to_msg()
        self.publisherArrow.publish(self.arrowMarker)
        
        self.circleMarker.header.stamp = self.get_clock().now().to_msg()
        self.circleMarker.pose.position.y = (self.circleMarker.pose.position.y + 0.1 ) % 3
        self.publisherCircle.publish(self.circleMarker)

        self.lineMarker.header.stamp = self.get_clock().now().to_msg()
        self.publisherLine.publish(self.lineMarker)  # Ajout de la publication du marqueur de ligne

        # self.randomMarker.header.stamp = self.get_clock().now().to_msg()
        # self.randomMarker.ns = f"random_ns_{random.randint(1, 100)}"
        # self.randomMarker.id = random.randint(0, 1000)
        # self.randomMarker.pose.position.x = random.uniform(-5.0, 5.0)
        # self.randomMarker.pose.position.y = random.uniform(-5.0, 5.0)
        # self.randomMarker.pose.position.z = random.uniform(-5.0, 5.0)
        # self.randomMarker.scale.x = random.uniform(0.1, 1.0)
        # self.randomMarker.scale.y = random.uniform(0.1, 1.0)
        # self.randomMarker.scale.z = random.uniform(0.1, 1.0)
        # self.randomMarker.color.r = random.uniform(0.0, 1.0)
        # self.randomMarker.color.g = random.uniform(0.0, 1.0)
        # self.randomMarker.color.b = random.uniform(0.0, 1.0)
        # self.randomMarker.color.a = 1.0
        # self.randomMarker.lifetime = Duration(sec=random.randint(1, 10))
        # self.publisherRandom.publish(self.randomMarker)


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
