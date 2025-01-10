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
        self.markers_list:List[MarkerRmv] = []
        self.node:Node = node
        
    def addMarker(self, marker:Marker)->None:
        """
        Add a new marker to the markers list.

        Args:
            marker (Marker): The marker to be added.
        """
        new_marker = MarkerRmv(marker)
        if self._replaceMarkerIfInList(new_marker):
            return
        self.markers_list.append(new_marker)
        
    def processNewMarkers(self, markers:list[Marker])->None:
        """
        Process a list of new markers.
        Args:
            markers (list[Marker]): The list of markers to be processed.
        """
        for marker in markers:
            self.addMarker(marker)
        self.deleteExpiredMarkers()

    def _replaceMarkerIfInList(self, new_marker:MarkerRmv)->bool:
        """
        Replace a marker in the markers list if it is already in the list.

        Args:
            new_marker (Marker): The marker to be replaced.
        return:
            bool: True if the marker was replaced, False otherwise.
        """
        for marker in self.markers_list:
            if marker.equals(new_marker):
                marker = new_marker
                return True
        return False
        
    def deleteExpiredMarkers(self):
        """
        Delete the expired markers from the markers list.
        """
        current_time = self.node.get_clock().now().to_msg()
        for marker_rmv in self.markers_list:
            if  marker_rmv.isExpired(current_time):
                self.markers_list.remove(marker_rmv)
                print(f"Marker {marker_rmv.getIdentifier()} removed")
        
    def getMarkersList(self)->List[MarkerRmv]:
        """
        Get the markers list.

        Returns:
            List[MarkerRmv]: The markers list.
        """
        return self.markers_list
    
    def cleanMarkersList(self):
        """
        Clean the markers list.
        """
        self.markers_list.clear()
    
    