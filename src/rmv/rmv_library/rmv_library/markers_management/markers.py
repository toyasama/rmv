from visualization_msgs.msg import Marker
from enum import Enum
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from abc import ABC, abstractmethod
from visualization_msgs.msg import Marker, MarkerArray
from typing import Type, List
from rclpy.node import Node
from threading import Thread, RLock
import time


class MarkerRmvBase:
    """Classe de base pour les objets Marker, gérant les aspects communs."""
    def __init__(self, marker: Marker, reception_time: Time):
        """
        Initialise un objet MarkerRmv de base.

        Args:
            marker (Marker): L'objet marker ROS.
            reception_time (Time): Le moment où le marker a été reçu.
        """
        self._marker = marker 
        self._pub_time = reception_time
        self._modified_pose = None  
        
    @property
    def identifier(self) -> tuple:
        """Retourne l'identifiant unique du marker (namespace, ID)."""
        return self._marker.ns, self._marker.id

    @property
    def pose(self) -> Pose:
        """Retourne la pose d'origine du marker."""
        return self._marker.pose
    
    @property
    def modified_pose(self) -> Pose:
        """Retourne la pose modifiée du marker (si définie)."""
        return self._modified_pose if self._modified_pose else self.pose
    
    @modified_pose.setter
    def modified_pose(self, new_pose: Pose) -> None:
        """Permet de modifier la pose modifiée du marker."""
        self._modified_pose = new_pose

    @property
    def scale(self):
        """Retourne la taille (scale) du marker."""
        return self._marker.scale
    
    @property
    def color(self):
        """Retourne la couleur du marker."""
        return self._marker.color
    
    @property
    def lifetime(self):
        """Retourne la durée de vie du marker."""
        return self._marker.lifetime

    @property
    def frame_id(self) -> str:
        """Retourne le cadre de référence (TF frame)."""
        return self._marker.header.frame_id
    
    @frame_id.setter
    def frame_id(self, frame_id: str):
        """Modifie le cadre de référence (TF frame)."""
        self._marker.header.frame_id = frame_id

    @property
    def points(self):
        """Retourne les points associés au marker."""
        return self._marker.points

    @property
    def type(self):
        """Retourne le type du marker."""
        return self._marker.type

    def getTransform(self):
        """Retourne la transformation du marker."""
        return self._marker.pose

    def isExpired(self, current_time: Time) -> bool:
        """Vérifie si le marker a expiré."""
        expiration_time = self.lifetime.sec + (self.lifetime.nanosec * 1e-9) + self._pub_time.sec + (self._pub_time.nanosec * 1e-9)
        current_time_in_seconds = current_time.sec + (current_time.nanosec * 1e-9)
        return current_time_in_seconds > expiration_time

1740216468.4591227,
1740216463.5565128
class MarkerRmv(MarkerRmvBase):
    """Classe spécifique à la gestion des markers avec identifiant et gestion du temps."""
    
    def __init__(self, marker: Marker, current_time: Time):
        """
        Initialise un MarkerRmv.

        Args:
            marker (Marker): Le marker à ajouter.
            current_time (Time): Le temps actuel du message.
        """
        super().__init__(marker, current_time) 
        
    def __eq__(self, value):
        if not isinstance(value, MarkerRmv):
            return False
        return self.identifier == value.identifier

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
        self.__new_markers:dict[tuple[str, int],MarkerRmv] = {}
        self.__new_msgs : List[MarkerArray|Marker] =[]
        self.__lock_markers_list = RLock()
        self.__lock_new_msgs = RLock()
        self.__running = True
        self.__delete_markers_thread :Thread = Thread(target=self._deleteExpiredMarkers).start()
        self.__merge_markers_thread :Thread = Thread(target=self.__mergeNewMarkers).start()
    
    def __mergeNewMarkers(self):
        while self.__running:
            self.__processMessage()
            with self.__lock_markers_list:
                self.__markers.update(self.__new_markers)
            self.__new_markers.clear()
            time.sleep(0.03)
    
    def __del__(self):
        self.__running = False
        self.__delete_markers_thread.join()
        self.__merge_markers_thread.join()
    
    def addMarker(self, marker: Marker | MarkerArray):
        """
        Add a new marker to the markers list.
        args:
            marker (Marker | MarkerArray): The marker to be added.
        """
        with self.__lock_new_msgs:
            self.__new_msgs.append(marker)
        
    def __processMessage(self):
        with self.__lock_new_msgs:
            new_msgs = self.__new_msgs.copy()
            self.__new_msgs.clear()
        reception_time = self.__node.get_clock().now().to_msg()
        for msg in new_msgs:
            if isinstance(msg, Marker):
                marker = MarkerMessage.process(msg, reception_time)
                if marker.isExpired(reception_time):
                    print(f"Marker {marker.identifier} expired before adding")
                    continue
                self.__new_markers[marker.identifier] = marker
                
            elif isinstance(msg, MarkerArray):
                marker_list:List[MarkerRmv] = MarkerArrayMessage.process(msg, reception_time)
                for marker in marker_list:
                    if marker.isExpired(reception_time):
                        print(f"Marker {marker.identifier} expired before adding")
                        continue
                    self.__new_markers[marker.identifier] = marker
            
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
        while self.__running:
            current_time = self.__node.get_clock().now().to_msg()
            with self.__lock_markers_list:
                expired_keys = [identifier for identifier, marker in self.__markers.items() if marker.isExpired(current_time)]
                for key in expired_keys:
                    del self.__markers[key]
            time.sleep(0.5)
