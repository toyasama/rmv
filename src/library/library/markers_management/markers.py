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



class MarkerRmvBase:
    """Classe de base pour les objets Marker, gérant les aspects communs."""
    def __init__(self, marker: Marker, reception_time: Time):
        """
        Initialise un objet MarkerRmv de base.

        Args:
            marker (Marker): L'objet marker ROS.
            reception_time (Time): Le moment où le marker a été reçu.
        """
        self._marker = marker  # Conserver l'objet Marker d'origine
        self._reception_time = reception_time  # Reception time
        self._modified_pose = None  # Pose modifiable temporaire (initialement None)
        
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
        expiration_time = self.lifetime.sec + (self.lifetime.nanosec * 1e-9) + self._reception_time.sec + (self._reception_time.nanosec * 1e-9)
        current_time_in_seconds = current_time.sec + (current_time.nanosec * 1e-9)
        return current_time_in_seconds > expiration_time


class MarkerRmv(MarkerRmvBase):
    """Classe spécifique à la gestion des markers avec identifiant et gestion du temps."""
    
    def __init__(self, marker: Marker, current_time: Time):
        """
        Initialise un MarkerRmv.

        Args:
            marker (Marker): Le marker à ajouter.
            current_time (Time): Le temps actuel du message.
        """
        super().__init__(marker, current_time)  # Héritage de MarkerRmvBase

    def equals(self, other_marker: 'MarkerRmv') -> bool:
        """
        Compare deux markers pour voir s'ils sont égaux (basé sur leur identifiant).

        Args:
            other_marker (MarkerRmv): Un autre marker à comparer.

        Returns:
            bool: True si les identifiants sont égaux, False sinon.
        """
        return self.identifier == other_marker.identifier

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
                self.__markers[MarkerMessage.process(marker, time).identifier] = MarkerMessage.process(marker, time)
        elif isinstance(marker, MarkerArray):
            for marker in MarkerArrayMessage.process(marker, time):
                with self.__lock_markers_list:
                    self.__markers[marker.identifier] = marker
            
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