from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Pose, TransformStamped, PoseStamped
from rclpy.time import Duration, Time
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
from typing import Optional, List
import math
from tf2_geometry_msgs import do_transform_pose
import yaml

class TFManager:
    def __init__(self, node: Node, buffer_timeout: float, cache_time:Duration) -> None:
        """
        Constructor for the TFManager class.
        Args:
            node (Node): The ROS2 node object.
            buffer_timeout (float): The timeout for TF lookup operations.
        """
        self.node = node
        self.buffer: Buffer = Buffer(cache_time=cache_time)
        self.listener = TransformListener(self.buffer, self.node, spin_thread=True)
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
        
    def getMainTfFrame(self) -> str:
        """
        Get the main TF frame for the TF manager.
        Returns:
            str: The name of the main TF frame.
        """
        return self.main_tf_frame
        
    def setDefaultMainFrame(self) -> None:
        """
        Set the main TF frame to the first available frame if ones.
        """
        frames = self.getAvailableFrames()
        if frames:
            self.main_tf_frame = frames[0]
    
    def isInMainTfFrame(self, frame: str) -> bool:
        """
        Check if a frame is the main TF frame.
        Args:
            frame (str): The frame name.
        Returns:
            bool: True if the frame is the main TF frame, False otherwise.
        """
        return frame == self.main_tf_frame
    

    def getAvailableFrames(self) -> List[str]:
        """
        List all unique TF frames, using YAML parsing for efficiency.

        Returns:
            List[str]: A clean list of unique frame names.
        """
        frames = set()
        try:
            yaml_frames = self.buffer.all_frames_as_yaml() 
            frame_data = yaml.safe_load(yaml_frames)
            for frame, data in frame_data.items():
                if not data["parent"]:
                    continue
                parent = data["parent"]
                frames.add(frame)
                frames.add(parent)
                

            return sorted(frames)  
        except Exception as e:
            # self.node.get_logger().error(f"Failed to get available frames: {str(e)}")
            return []
    


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
            return False

        try:
            return self.buffer.can_transform(target_frame, source_frame, self.buffer_timeout)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.node.get_logger().error(f"Failed to check transform between {source_frame} and {target_frame}: {e}")
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
            self.node.get_logger().warning("Source or target frame is empty. Transformation aborted.")
            return None

        now = self.node.get_clock().now()
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = now.to_msg()
            pose_stamped.header.frame_id = source_frame
            pose_stamped.pose = pose

            transformed_pose_stamped: PoseStamped = self.buffer.transform(pose_stamped, target_frame,timeout=self.buffer_timeout)

            return transformed_pose_stamped.pose

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.node.get_logger().error(f"Failed to transform pose from {source_frame} to {target_frame}: {str(e)}")
            return None

        
