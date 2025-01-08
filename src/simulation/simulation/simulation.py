import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MarkersPublisher(Node):
    def __init__(self):
        super().__init__('markers_node')
        self.publisher_cube = self.create_publisher(Marker, '/cube_marker', 10)
        self.publisher_sphere = self.create_publisher(Marker, '/sphere_marker', 10)
        self.publisher_line = self.create_publisher(Marker, '/line_marker', 10)

        timer_period = 1.0  # 1 second
        self.timer = self.create_timer(timer_period, self.publish_markers)
        self.get_logger().info("MarkersPublisher initialized and running.")

    def publish_markers(self):
        # Cube
        cube_marker = Marker()
        cube_marker.header.frame_id = "base_link"
        cube_marker.header.stamp = self.get_clock().now().to_msg()
        cube_marker.ns = "cube"
        cube_marker.id = 0
        cube_marker.type = Marker.CUBE
        cube_marker.action = Marker.ADD
        cube_marker.pose.position.x = 1.0
        cube_marker.pose.position.y = 0.0
        cube_marker.pose.position.z = 0.0
        cube_marker.pose.orientation.x = 0.0
        cube_marker.pose.orientation.y = 0.0
        cube_marker.pose.orientation.z = 0.0
        cube_marker.pose.orientation.w = 1.0
        cube_marker.scale.x = 0.5
        cube_marker.scale.y = 0.5
        cube_marker.scale.z = 0.5
        cube_marker.color.r = 1.0
        cube_marker.color.g = 0.0
        cube_marker.color.b = 0.0
        cube_marker.color.a = 1.0
        self.publisher_cube.publish(cube_marker)

        # Sphere
        sphere_marker = Marker()
        sphere_marker.header.frame_id = "base_link"
        sphere_marker.header.stamp = self.get_clock().now().to_msg()
        sphere_marker.ns = "sphere"
        sphere_marker.id = 1
        sphere_marker.type = Marker.SPHERE
        sphere_marker.action = Marker.ADD
        sphere_marker.pose.position.x = 2.0
        sphere_marker.pose.position.y = 0.0
        sphere_marker.pose.position.z = 0.0
        sphere_marker.scale.x = 0.5
        sphere_marker.scale.y = 0.5
        sphere_marker.scale.z = 0.5
        sphere_marker.color.r = 0.0
        sphere_marker.color.g = 1.0
        sphere_marker.color.b = 0.0
        sphere_marker.color.a = 1.0
        self.publisher_sphere.publish(sphere_marker)

        # Line
        line_marker = Marker()
        line_marker.header.frame_id = "base_link"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "line"
        line_marker.id = 2
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.points = [Point(x=3.0, y=0.0, z=0.0), Point(x=3.0, y=1.0, z=0.0)]
        line_marker.scale.x = 0.1
        line_marker.color.r = 0.0
        line_marker.color.g = 0.0
        line_marker.color.b = 1.0
        line_marker.color.a = 1.0
        self.publisher_line.publish(line_marker)

        self.get_logger().info("Published markers: Cube, Sphere, and Line.")

def main(args=None):
    rclpy.init(args=args)
    node = MarkersPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
