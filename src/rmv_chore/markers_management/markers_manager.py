from markers_management.markers import MarkerRmv
from visualization_msgs.msg import Marker
from typing import List
from rclpy.node import Node

class MarkersManager:
    def __init__(self,node)->None:
        """
        Constructor for the MarkersManager class.
        Args:
            node (Node): The node object.
        """
        self.markers_list:dict[tuple[str, int],MarkerRmv] = {}
        self.node:Node = node
        
    def addMarker(self, marker:Marker)->None:
        """
        Add a new marker to the markers list.

        Args:
            marker (Marker): The marker to be added.
        """
        new_marker = MarkerRmv(marker, self.node.get_clock().now().to_msg())
        self.markers_list[new_marker.getIdentifier()] = new_marker
        
    def processMarkers(self, markers:list[Marker])->None:
        """
        Process a list of new markers.
        Args:
            markers (list[Marker]): The list of markers to be processed.
        """
        for marker in markers:
            self.addMarker(marker)
        self.deleteExpiredMarkers()

        
    def deleteExpiredMarkers(self):
        """
        Delete the expired markers from the markers list.
        """
        current_time = self.node.get_clock().now().to_msg()
        self.markers_list = {
            identifier: marker
            for identifier, marker in self.markers_list.items()
            if not marker.isExpired(current_time)
        }
        
    def getMarkersList(self)->List[MarkerRmv]:
        """
        Get the markers list.

        Returns:
            List[MarkerRmv]: The markers list.
        """
        return list(self.markers_list.values())
    
    def cleanMarkersList(self):
        """
        Clean the markers list.
        """
        self.markers_list.clear()
    
    