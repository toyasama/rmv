from visualization_msgs.msg import Marker
from enum import Enum
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from abc import ABC, abstractmethod
from visualization_msgs.msg import Marker, MarkerArray
from typing import Type, List
from rclpy.node import Node
from threading import Thread, RLock
from time import sleep

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
    def __init__(self,  marker: Marker, current_time: Time):
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
        self.type = marker.type
        self.reception_time = current_time
        self.points = marker.points

    def getType(self):
        return self.type


    def getIdentifier(self) -> tuple:
        """
        Get the marker's identifier.

        Returns:
            tuple: A tuple containing the namespace and ID.
        """
        return self.identifier
    
    def getTfFrame(self) -> str:
        """
        Get the TF frame of the marker.

        Returns:
            str: The TF frame of the marker.
        """
        return self.header.frame_id
    
    def getPose(self) -> Pose:
        """
        Get the pose of the marker.

        Returns:
            Pose: The pose of the marker.
        """
        return self.pose
    
    def setFrameId(self, frame_id: str) -> None:
        """
        Set the TF frame of the marker.

        Args:
            frame_id (str): The new TF frame.
        """
        self.header.frame_id = frame_id
        
    def setPose(self, pose: Pose) -> None:
        """
        Set the pose of the marker.

        Args:
            pose (Pose): The new pose.
        """
        self.pose = pose

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
        return current_time.sec + (current_time.nanosec * 1e-9) > self.lifetime.sec + (self.lifetime.nanosec * 1e-9) + self.reception_time.sec + (self.reception_time.nanosec * 1e-9)
    

class BaseMessage(ABC):
    def __init__(self, message_type: Type[Marker | MarkerArray]):
        self.message_type = message_type
    @abstractmethod
    def process(self, message, time: Time):
        """Do something with the message."""
        pass

class MarkerMessage(BaseMessage):
    def __init__(self):
        super().__init__(Marker)
    @staticmethod
    def process( message: Marker, time: Time)-> MarkerRmv:
        return MarkerRmv(message, time)

class MarkerArrayMessage(BaseMessage):
    def __init__(self):
        super().__init__(MarkerArray)
    @staticmethod
    def process( message: MarkerArray, time: Time) -> List[MarkerRmv]:
        return [MarkerRmv(marker, time) for marker in message.markers]
    
class MarkersHandler:
    def __init__(self, node: Node):
        self.__node = node
        self.__markers:dict[tuple[str, int],MarkerRmv] = {}
        self.__lock_markers_list = RLock()
        self.__running = True
        self.__thread :Thread = Thread(target=self._deleteExpiredMarkers).start()
    
    def __del__(self):
        self.__running = False
        self.__thread.join()
    
    def addMarker(self, marker: Marker | MarkerArray):
        """
        Add a new marker to the markers list.
        args:
            marker (Marker | MarkerArray): The marker to be added.
        """
        time = self.__node.get_clock().now().to_msg()
        if isinstance(marker, Marker):
            with self.__lock_markers_list:
                self.__markers[MarkerMessage.process(marker, time).getIdentifier()] = MarkerMessage.process(marker, time)
        elif isinstance(marker, MarkerArray):
            for marker in MarkerArrayMessage.process(marker, time):
                with self.__lock_markers_list:
                    self.__markers[marker.getIdentifier()] = marker
            
    @property
    def markers(self) ->List[MarkerRmv]:
        return list(self.__markers.values())

    def clearMarkersList(self):
        """
        Clear the markers list.
        """
        with self.__lock_markers_list:
            self.__markers.clear()
    
    def _deleteExpiredMarkers(self):
        """
        Delete the expired markers from the markers list.
        """
        current_time = self.__node.get_clock().now().to_msg()
        while self.__running:
            with self.__lock_markers_list:
                self.__markers = {
                    identifier: marker
                    for identifier, marker in self.__markers.items()
                    if not marker.isExpired(current_time)
                }
            sleep(1)