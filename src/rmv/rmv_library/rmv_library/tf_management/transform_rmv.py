from geometry_msgs.msg import Transform, TransformStamped
import time

class TransformBase:
    __expiration_duration = 3.0
    def __init__(self, transform_stamped: TransformStamped, static: bool = False):
        self._name: str = transform_stamped.child_frame_id
        self._parent: str = transform_stamped.header.frame_id
        self._transform: Transform = transform_stamped.transform
        self._timestamp: float = transform_stamped.header.stamp.sec + transform_stamped.header.stamp.nanosec * 1e-9
        self._static = static
        
    @property
    def name(self) -> str:
        return self._name
    
    @property
    def parent(self) -> str:
        return self._parent
    
    @property
    def transform(self) -> Transform:
        return self._transform

    @property
    def opacity(self) -> float:
        if self._static:
            return 1.0
        elapsed_time = time.time() - self._timestamp
        return max(0,1.0 - elapsed_time / self.__expiration_duration)
    
    @property
    def isExpired(self) -> bool:
        return not self._static and time.time() > self._timestamp + self.__expiration_duration
    
    def update(self, transform_stamped: TransformStamped) -> None:
        if not self == transform_stamped:
            print("Error: Trying to update a transform with different name or parent.")
            print(f"Old transform: {self.name} -> {self.parent}")
            print(f"New transform: {transform_stamped.child_frame_id} -> {transform_stamped.header.frame_id}")
            return
        self._timestamp = transform_stamped.header.stamp.sec + transform_stamped.header.stamp.nanosec * 1e-9
        self._transform = transform_stamped.transform
        
        
    def __eq__(self, value) -> bool:
        if isinstance(value, TransformBase):
            return self.name == value.name and self.parent == value.parent
        elif isinstance(value, TransformStamped):
            return self.name == value.child_frame_id and self.parent == value.header.frame_id
        return False
        
class TransformDrawerInfo():
    def __init__(self,Transform_base: TransformBase):
        self._main_frame: str = ""
        self._pose_in_main_frame: Transform = Transform()
        self._start_connection: Transform = Transform()
        self._end_connection: Transform = Transform()
        self.__associated_transform: TransformBase = Transform_base
    
    @property
    def transform_name(self) -> str:
        return self.__associated_transform.name
    
    @property
    def parent(self) -> str:
        return self.__associated_transform.parent
    
    @property  
    def pose_in_main_frame(self) -> Transform:
        return self._pose_in_main_frame
        
    @property
    def main_frame(self) -> str:
        return self._main_frame
        
    @property
    def start_connection(self) -> Transform:
        return self._start_connection
    
    @property
    def end_connection(self) -> Transform:
        return self._end_connection
    
    @property
    def opacity(self) -> float:
        return self.__associated_transform.opacity
    
    def toDraw(self, main_frame) -> bool:
        return (self._main_frame == main_frame) and not self.__associated_transform.isExpired
        
    
    def update(self, main_frame: str,pose_in_main_frame:Transform,  start_connection: Transform, end_connection: Transform) -> None:
        self._main_frame = main_frame
        self._pose_in_main_frame = pose_in_main_frame
        self._start_connection = start_connection
        self._end_connection = end_connection
    
    def __str__(self) -> str:
        return f"Main frame: {self._main_frame}, Start connection: {self._start_connection.translation}, End connection: {self._end_connection.translation}"
    
    def __repr__(self):
        return self.__str__()
    
class RmvTransform(TransformBase):
    def __init__(self,  transform_stamped: TransformStamped, static: bool ):
        super().__init__(transform_stamped, static)
        self._drawer_info = TransformDrawerInfo(self)    
        self._initial_direction = True
    
    @property
    def drawer_info(self) -> TransformDrawerInfo:
        return self._drawer_info
    
    def updateTransformDrawerInfo(self, main_frame: str,pose_in_main_frame:Transform,  start_connection: Transform, end_connection: Transform) -> None:
        self._drawer_info.update(main_frame,pose_in_main_frame, start_connection, end_connection)
    
    def __eq__(self, value) -> bool:
        return super().__eq__(value) 
    
    def __hash__(self) -> int:
        return hash((self.name, self.parent))