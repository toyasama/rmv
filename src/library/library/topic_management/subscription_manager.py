from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.subscription import Subscription
from visualization_msgs.msg import Marker, MarkerArray
from typing import Type, List, Dict
from ..markers_management.markers import  MarkersHandler
from ..utils.timer_log import TimerLogger

class SubscriptionManager():
    def __init__(self, node: Node, markers_handler: MarkersHandler):
        """
        Manager for handling topic __subscriptions.
        Args:
            node (Node): The ROS2 node object.
        """
        self._node = node
        self.__subscriptions: Dict[str, Subscription] = {}
        self.timer_logger : TimerLogger= TimerLogger( node, 5.0)
        self.marker_handler = markers_handler

    @property
    def active_topics(self) -> List[str]:
        """
        Get a list of currently active topics.
        Returns:
            list: List of topic names.
        """
        return list(self.__subscriptions.keys())
    
    def subscribe(self, topic: str, topic_type: str):
        """
        Subscribe to a new topic if not already subscribed.
        Args:
            topic (str): Topic name.
            topic_type (str): ROS2 message type string.
            callback: Callback function for the topic.
        """
        if topic not in list(self.__subscriptions.keys()):
            print(f"Subscribe to {topic}")
            message_type = self._getMessageType(topic_type)
            self.__subscriptions[topic] = self._node.create_subscription(message_type, topic, self.callback, qos_profile_sensor_data)

    def callback(self, message: Marker | MarkerArray):
        """
        Callback function for the subscribed topic.
        Args:
            message (Marker | MarkerArray): The received message.
        """
        self.marker_handler.addMarker(message)
        # self.timer_logger.logExecutionTime(self.addMarker)(message)

    def unsubscribe(self, topic: str):
        """
        Unsubscribe from a topic.
        Args:
            topic (str): Topic name.
        """
        if topic in self.__subscriptions:
            self._node.get_logger().info(f"Unsubscribing from: {topic}")
            del self.__subscriptions[topic]


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
