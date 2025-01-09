from visualization_msgs.msg import Marker
from enum import Enum
from builtin_interfaces.msg import Time

class MarkerTypes(Enum):
    """
    Enum for marker types.
    """
    ARROW = 0
    CUBE = 1
    SPHERE = 2
    CYLINDER = 3
    LINE_STRIP = 4
    LINE_LIST = 5
    CUBE_LIST = 6
    SPHERE_LIST = 7
    POINTS = 8
    TEXT_VIEW_FACING = 9
    MESH_RESOURCE = 10
    TRIANGLE_LIST = 11
    
class MarkerRmv:
    def __init__(self,  marker: Marker):
        """
        Constructor for the MarkerRmv class with namespace and ID.

        Args:
            marker (Marker): The marker to be added.
        """
        self.identifier = (marker.ns, marker.id) 
        self.pose = marker.pose
        self.scale = marker.scale
        self.color = marker.color
        self.lifetime = marker.lifetime
        self.header = marker.header


    def getIdentifier(self) -> tuple:
        """
        Get the marker's identifier.

        Returns:
            tuple: A tuple containing the namespace and ID.
        """
        return self.identifier

    def equals(self, other_marker: 'MarkerRmv') -> bool:
        """
        Check if another marker has the same identifier.

        Args:
            other_marker (MarkerRmv): Another marker to compare.

        Returns:
            bool: True if the identifiers match, False otherwise.
        """
        return self.identifier == other_marker.getIdentifier()

    def isExpired(self, current_time: Time) -> bool:
        """
        Check if a marker is expired.

        Args:
            current_time (Time): The current time.

        Returns:
            bool: True if the marker is expired, False otherwise.
        """
        return current_time.sec >= self.lifetime.sec + self.header.stamp.sec
    
    def getFrameId(self) -> str:
        """
        Get the frame ID of the marker.

        Returns:
            str: The frame ID of the marker.
        """
        return self.header.frame_id
