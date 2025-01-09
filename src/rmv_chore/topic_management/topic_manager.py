from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from typing import Type, Tuple, List, Dict
from utils.timer_log import TimerLogger
from topic_management.subscription_manager import SubscriptionManager

class TopicManager:
    def __init__(self, node: Node) -> None:
        """
        Constructor for the TopicManager class.
        Args:
            node (Node): The node object.
        """
        self.node = node
        self.markers_list: List[Type[Marker|MarkerArray]] = []
        self.subscription_manager = SubscriptionManager(node)
        self.timer_logger = TimerLogger(node)
        self.node.create_timer(0.25, self.findMarkersTopicsCallBack) 
        self.node.get_logger().info("TopicManager created successfully")

    def findMarkersTopicsCallBack(self) -> None:
        """
        Discover topics related to markers, subscribe to new ones, and remove obsolete subscriptions.
        """
        self.timer_logger.logExecutionTime(self._findMarkersTopics)()
        
    def _findMarkersTopics(self):
        """
        Discover topics related to markers, subscribe to new ones, and remove obsolete subscriptions.
        This method will be decorated to log execution time.
        """
        topics_and_types = self.node.get_topic_names_and_types()
        expected_types = ["visualization_msgs/msg/Marker", "visualization_msgs/msg/MarkerArray"]
        filtered_topics = self._filterAndValidateTopics(topics_and_types, expected_types)

        self._removeUnpublishedTopics(filtered_topics)
        self._subscribeToTopics(filtered_topics)

    def _filterAndValidateTopics(
        self, topics: List[Tuple[str, List[str]]], expected_types: List[str]
    ) -> List[Tuple[str, str]]:
        """
        Filter topics based on expected types and active publishers.
        Args:
            topics (list): A list of tuples containing topic names and their types.
            expected_types (list): A list of expected message types.
        Returns:
            list: A list of tuples with topic names and matching types.
        """
        return [
            (topic, matching_type)
            for topic, types in topics
            if (matching_type := self._hasMatchingType(types, expected_types)) and self._hasPublisher(topic)
        ]

    def _subscribeToTopics(self, filtered_topics: List[Tuple[str, str]]) -> None:
        """
        Subscribe to topics related to markers.
        Args:
            filtered_topics (list): The list of currently available marker topics.
        """
        for topic, topic_type in filtered_topics:
            self.subscription_manager.subscribe(topic, topic_type, self._markerCallback)

    def _removeUnpublishedTopics(self, filtered_topics: List[Tuple[str, str]]) -> None:
        """
        Remove subscriptions to topics that are no longer published or have no active publishers.
        Args:
            filtered_topics (list): The list of currently available marker topics.
        """
        activeTopics = {topic for topic, _ in filtered_topics}
        for subscribed_topic in self.subscription_manager.activeTopics():
            if subscribed_topic not in activeTopics:
                self.subscription_manager.unsubscribe(subscribed_topic)

    def _hasMatchingType(self, received_types: List[str], expected_types: List[str]) -> str | None:
        """
        Check if any of the received types match one of the expected types.
        Args:
            received_types (list): A list of types associated with a topic.
            expected_types (list): A list of expected types.
        Returns:
            str | None: The matching type if found, otherwise None.
        """
        return next((t for t in received_types if t in expected_types), None)

    def _hasPublisher(self, topic: str) -> bool:
        """
        Check if a topic has active publishers.
        Args:
            topic (str): The topic name to check.
        Returns:
            bool: True if the topic has at least one publisher, False otherwise.
        """
        return self.node.count_publishers(topic) > 0

    def _markerCallback(self, msg: Marker | MarkerArray) -> None:
        """
        Callback for receiving marker messages.
        Args:
            msg (Marker | MarkerArray): The received message.
        """
        self.markers_list.append(msg)
        
    def getMarkersList(self) -> List[Type[Marker|MarkerArray]]:
        """
        Get the list of markers.
        Returns:
            list: The list of markers.
        """
        return self.markers_list
    
        
    def cleanMarkersList(self) -> None:
        """
        Clean the list of markers.
        """
        self.markers_list = []
        
