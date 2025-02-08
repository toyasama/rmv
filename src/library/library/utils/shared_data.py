import threading
from ..markers_management.markers import MarkerRmv
from ..tf_management.tf import FrameDrawingInfo
from typing import List, Dict
from geometry_msgs.msg import TransformStamped
class SharedData:
    def __init__(self):
        self.main_tf : FrameDrawingInfo = FrameDrawingInfo()   # Stocke la transformation principale
        self.other_tfs: Dict[str, TransformStamped] = {}
        self.markers: List[MarkerRmv] = []

    def update_main_tf(self, tf: FrameDrawingInfo):
       self.main_tf = tf

    def update_other_tfs(self, tfs: List[FrameDrawingInfo]):
       self.other_tfs = tfs

    def update_markers(self, markers: List[MarkerRmv]):
       self.markers = markers

    def get_main_tf(self)-> FrameDrawingInfo:
       return self.main_tf

    def get_other_tfs(self) -> List[FrameDrawingInfo]:
       return self.other_tfs

    def get_markers(self):
       return self.markers
