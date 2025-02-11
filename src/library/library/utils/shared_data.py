import threading
from ..markers_management.markers import MarkerRmv
from ..tf_management.tf import FrameRMV
from typing import List, Dict
from geometry_msgs.msg import TransformStamped
from threading import Lock
class SharedData:
    def __init__(self):
      self.main_tf : FrameRMV = FrameRMV()   # Stocke la transformation principale
      self.other_tfs: Dict[str, TransformStamped] = {}
      self.markers: List[MarkerRmv] = []
      self.lock_main_tf = Lock()
      self.lock_other_tf = Lock()
      self.lock_markers = Lock()

    def update_main_tf(self, tf: FrameRMV):
       with self.lock_main_tf:
         self.main_tf = tf

    def update_other_tfs(self, tfs: List[FrameRMV]):
       with self.lock_other_tf:
         self.other_tfs = tfs

    def update_markers(self, markers: List[MarkerRmv]):
      with self.lock_markers:
         self.markers = markers

    def get_main_tf(self)-> FrameRMV:
       with self.lock_main_tf:
         return self.main_tf

    def get_other_tfs(self) -> List[FrameRMV]:
       with self.lock_other_tf:
         return self.other_tfs

    def get_markers(self):
      with self.lock_markers:
         return self.markers
