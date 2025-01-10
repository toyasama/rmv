from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.time import Duration, Time
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
from typing import Optional, List
import math
class TFManager:
    def __init__(self, node: Node, buffer_timeout: float) -> None:
        """
        Constructor for the TFManager class.
        Args:
            node (Node): The ROS2 node object.
            buffer_timeout (float): The timeout for TF lookup operations.
        """
        self.node = node
        self.buffer: Buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        seconds = math.floor(buffer_timeout)  
        nanoseconds = int((buffer_timeout - seconds) * 1e9)  
        self.buffer_timeout: Duration = Duration(seconds=seconds, nanoseconds=nanoseconds)
        self.main_tf_frame = ""
        self.node.get_logger().info("TFManager initialized successfully.")
        
    def setMainTfFrame(self, frame: str) -> None:
        """
        Set the main TF frame for the TF manager.
        Args:
            frame (str): The name of the main TF frame.
        """
        self.main_tf_frame = frame
        
    def setDefaultMainFrame(self) -> None:
        """
        Set the main TF frame to the first available frame if ones.
        """
        frames = self.getAvailableFrames()
        if frames:
            self.main_tf_frame = frames[0]
            
    def getAvailableFrames(self) -> List[str]:
        """
        List all available TF frames.
        Returns:
            List[str]: A list of available frame names.
        """
        return self.buffer.all_frames_as_string().split("\n")

    def canTransform(self, source_frame: str, target_frame: str) -> bool:
        """
        Check if a transformation is possible between two frames.
        Args:
            source_frame (str): The source frame name.
            target_frame (str): The target frame name.
        Returns:
            bool: True if the transformation is possible, False otherwise.
        """
        if not source_frame or not target_frame:
            return None
        now = self.node.get_clock().now()
        try:
            self.buffer.lookup_transform(target_frame, source_frame, now, self.buffer_timeout)
            return True
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False

    def transformPose(self, pose: Pose, source_frame: str, target_frame: str) -> Optional[Pose]:
        """
        Transform a pose from one frame to another.
        Args:
            pose (Pose): The pose to be transformed.
            source_frame (str): The source frame name.
            target_frame (str): The target frame name.
        Returns:
            Optional[Pose]: The transformed pose, or None if the transformation fails.
        """
        if not source_frame or not target_frame:
            return None
        now = self.node.get_clock().now()
        try:
            transform :TransformStamped= self.buffer.lookup_transform(target_frame, source_frame, now, self.buffer_timeout)
            transformed_pose = self.buffer.transform(pose, transform)
            return transformed_pose
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        
        
"""  try:
    transform: TransformStamped = self.buffer.lookup_transform(
        target_frame, source_frame, 0, self.buffer_timeout
    )
    transformed_point = Point()
    transformed_point.x = (
        transform.transform.translation.x +
        point.x * transform.transform.rotation.w - 
        point.y * transform.transform.rotation.z + 
        point.z * transform.transform.rotation.y
    )
    transformed_point.y = (
        transform.transform.translation.y +
        point.x * transform.transform.rotation.z +
        point.y * transform.transform.rotation.w - 
        point.z * transform.transform.rotation.x
    )
    transformed_point.z = (
        transform.transform.translation.z -
        point.x * transform.transform.rotation.y +
        point.y * transform.transform.rotation.x +
        point.z * transform.transform.rotation.w
    )
    return transformed_point
except (LookupException, ConnectivityException, ExtrapolationException) as e:
    self.node.get_logger().warn(f"Failed to transform point: {e}")
    return None """
