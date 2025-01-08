
from rclpy.node import Node
from topic_management.topic_manager import TopicManager

class RmvChore:
    def __init__(self)->None:
        """
        Constructor for the RmvChore class.
        """
        self.node : Node= Node("rmv_chore")
        self.topic_manager : TopicManager = TopicManager(self.node)
        print("Node rmv_chore created successfully")
        
        
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
