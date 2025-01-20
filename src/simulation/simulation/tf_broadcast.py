import rclpy
from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import random
random.seed()
import numpy as np

class StaticTFPublisher(Node):
    def __init__(self)->None:
        """
        Constructor for the StaticTFPublisher class.
        """
        super().__init__('static_tf_publisher')
        self.tfBroadcaster = TransformBroadcaster(self)
        self.staticTransforms = self.initializeTransforms()
        self.timer = self.create_timer(1.0, self.publishStaticTransforms)
        self.timerFrame3 = self.create_timer(1.0, self.publishFrame3)
        self.get_logger().info("StaticTFPublisher initialized and running.")

    def initializeTransforms(self)->list[TransformStamped]:
        """ 
        Initialize the static transforms for the simulation.
        Returns:
            List[TransformStamped]: A list of static transforms."""
        transforms = []
        transform1To2 = self.createTransform("frame_1", "frame_2", x=1.0, y=0.0, z=0.0)
        transforms.append(transform1To2)
        self.frame3Transform = self.createTransform("frame_1", "frame_3", x=2.0, y=1.0, z=0.0)
        transform1To4 = self.createTransform("frame_1", "frame_4", x=0.5, y=2.0, z=1.0)
        transforms.append(transform1To4)
        return transforms

    def createTransform(self, parentFrame, childFrame, x=0.0, y=0.0, z=0.0)->TransformStamped:
        """ 
        Create a static transform between two frames.
        Args:
            parentFrame (str): The parent frame name.
            childFrame (str): The child frame name.
            x (float): The x-axis translation.
            y (float): The y-axis translation.
            z (float): The z-axis translation.
        Returns:
            TransformStamped: The static transform.
        """
        theta = np.random.uniform(0, 2 * np.pi)
        transform = TransformStamped()
        transform.header.frame_id = parentFrame
        transform.child_frame_id = childFrame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.x = 0.
        transform.transform.rotation.y = 0.
        transform.transform.rotation.z = 0. # np.sin(theta / 2)
        transform.transform.rotation.w = 1. # np.cos(theta / 2)
        return transform

    def publishStaticTransforms(self)->None:
        """
        Publish the static transforms.
        """
        for transform in self.staticTransforms:
            transform.header.stamp = self.get_clock().now().to_msg()
        self.tfBroadcaster.sendTransform(self.staticTransforms)

    def publishFrame3(self)->None:
        """
        Publish the frame_3 static transform.
        """
        self.frame3Transform.header.stamp = self.get_clock().now().to_msg()
        self.tfBroadcaster.sendTransform([self.frame3Transform])

def main(args=None):
    rclpy.init(args=args)
    node = StaticTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
