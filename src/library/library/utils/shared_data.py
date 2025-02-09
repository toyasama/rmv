import threading
from ..markers_management.markers import MarkerRmv
from ..tf_management.tf import FrameRMV
from typing import List, Dict
from geometry_msgs.msg import TransformStamped
class SharedData:
    def __init__(self):
        self.main_tf : FrameRMV = FrameRMV()   # Stocke la transformation principale
        self.other_tfs: Dict[str, TransformStamped] = {}
        self.markers: List[MarkerRmv] = []

    def update_main_tf(self, tf: FrameRMV):
       self.main_tf = tf

    def update_other_tfs(self, tfs: List[FrameRMV]):
       self.other_tfs = tfs

    def update_markers(self, markers: List[MarkerRmv]):
       self.markers = markers

    def get_main_tf(self)-> FrameRMV:
       return self.main_tf

    def get_other_tfs(self) -> List[FrameRMV]:
       return self.other_tfs

    def get_markers(self):
       return self.markers
