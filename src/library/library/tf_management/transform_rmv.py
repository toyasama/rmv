from geometry_msgs.msg import Transform
import time
from .transform_utils import  TransformUtils

class TransformRMV():
    """
    Classe représentant une transformation avec métadonnées.
    """
    def __init__(self, parent: str, child: str, transform: Transform):
        self._parent = parent
        self._child = child
        self._transform = transform
        self._static = False
        self._expiration_duration = 2.0  # Durée d'expiration par défaut
        self._validity_duration = 0.2  # Durée de validité par défaut
        self._initial_direction = None
        self.resetValidity()
    
    @property
    def initial_direction(self) -> bool:
        return self._initial_direction
    
    @initial_direction.setter
    def initial_direction(self, value: bool) -> None:
        self._initial_direction = value
    
    @property
    def isStatic(self) -> bool:
        return self._static
    
    @isStatic.setter
    def isStatic(self, value: bool) -> None:
        self._static = value
        self._expiration_time = None if value else time.time() + self._expiration_duration
    
    @property
    def expiration_duration(self) -> float:
        return self._expiration_duration
    
    @expiration_duration.setter
    def expiration_duration(self, duration: float) -> None:
        self._expiration_duration = duration
        if not self._static:
            self._expiration_time = time.time() + duration
    
    @property
    def validity_duration(self) -> float:
        return self._validity_duration
    
    @validity_duration.setter
    def validity_duration(self, duration: float) -> None:
        self._validity_duration = duration
        self.resetValidity()
    
    def resetValidity(self) -> None:
        self._validity_time = time.time() + self._validity_duration
    
    @property
    def isExpired(self) -> bool:
        return not self._static and time.time() > self._expiration_time
    
    @property
    def isValid(self) -> bool:
        return time.time() < self._validity_time
    
    @property
    def opacity(self) -> float:
        if self._static:
            return 1.0
        if self.isExpired:
            return 0.0
        return (self._expiration_time - time.time()) / self._expiration_duration
    
    @property
    def parent_name(self) -> str:
        return self._parent
    
    def getTransform(self, inverse: bool = False) -> Transform:
        return TransformUtils.invertTransform(self._transform) if inverse else self._transform