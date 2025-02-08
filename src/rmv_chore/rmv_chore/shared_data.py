import threading
from typing import Dict, List
from markers_management.markers import MarkerRmv
from geometry_msgs.msg import TransformStamped
from tf_management.graph import FrameDrawingInfo
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
