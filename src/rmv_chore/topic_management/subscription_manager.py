from rclpy.node import Node
from rclpy.subscription import Subscription
from visualization_msgs.msg import Marker, MarkerArray
from typing import Type, List, Dict

class SubscriptionManager:
    def __init__(self, node: Node):
        """
        Manager for handling topic subscriptions.
        Args:
            node (Node): The ROS2 node object.
        """
        self.node = node
        self.subscriptions: Dict[str, Subscription] = {}

    def subscribe(self, topic: str, topic_type: str, callback):
        """
        Subscribe to a new topic if not already subscribed.
        Args:
            topic (str): Topic name.
            topic_type (str): ROS2 message type string.
            callback: Callback function for the topic.
        """
        if topic not in self.subscriptions:
            self.node.get_logger().info(f"Subscribing to: {topic} ({topic_type})")
            message_type = self._getMessageType(topic_type)
            self.subscriptions[topic] = self.node.create_subscription(message_type, topic, callback, 10)

    def unsubscribe(self, topic: str):
        """
        Unsubscribe from a topic.
        Args:
            topic (str): Topic name.
        """
        if topic in self.subscriptions:
            self.node.get_logger().info(f"Unsubscribing from: {topic}")
            del self.subscriptions[topic]

    def activeTopics(self) -> List[str]:
        """
        Get a list of currently active topics.
        Returns:
            list: List of topic names.
        """
        return list(self.subscriptions.keys())

    def _getMessageType(self, type_string: str) -> Type[Marker | MarkerArray]:
        """
        Get the ROS2 message class based on the type string.
        Args:
            type_string (str): The ROS2 type string (e.g., "visualization_msgs/msg/Marker").
        Returns:
            Type[Marker | MarkerArray]: The corresponding message class.
        Raises:
            ValueError: If the type string is invalid or unsupported.
        """
        message_type_map = {
            "Marker": Marker,
            "MarkerArray": MarkerArray
        }
        message_type = type_string.split('/')[-1]
        if message_type in message_type_map:
            return message_type_map[message_type]
        else:
            raise ValueError(f"Unsupported type: {message_type}")

