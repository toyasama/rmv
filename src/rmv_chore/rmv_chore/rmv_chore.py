
from rclpy.node import Node
from topic_management.topic_manager import TopicManager
from markers_management.markers_manager import MarkersManager
from pprint import pprint
class RmvChore:
    def __init__(self)->None:
        """
        Constructor for the RmvChore class.
        """
        self.node : Node= Node("rmv_chore")
        self.topic_manager : TopicManager = TopicManager(self.node)
        self.timer_test = self.node.create_timer(0.25, self.timer_test_callback)
        self.markers_manager = MarkersManager(self.node)
        print("Node rmv_chore created successfully")
        
    def timer_test_callback(self)->None:
        """
        Timer callback method.
        """
        markers = self.topic_manager.getMarkersList()
        self.topic_manager.cleanMarkersList()
        for marker in markers:
            self.markers_manager.addMarker(marker)
        self.markers_manager.deleteExpiredMarkers()
        pprint([marker.getIdentifier() for marker in self.markers_manager.getMarkersList()])
        
    def getNode(self)->Node:
        """
        Method to get the node object.
        Returns:
            Node: The node object.
        """
        return self.node
    
    def destroyNode(self)->None:
        """
        Method to destroy the node object.
        """
        self.node.destroy_node()
        print("Node rmv_chore destroyed")
