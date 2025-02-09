from typing import List
from geometry_msgs.msg import Transform

class FrameDrawingInfo:
    def __init__(self):
        """Information for drawing a TF."""
        self.name : str = ""
        self.transform : Transform = Transform()
        self.opacity : float = 0.0
        self.start_connection : Transform = Transform()
        self.end_connection : Transform = Transform()
        self.valid : bool = False
    @classmethod
    def fill(cls, frame: str, transform: Transform, start_connection: Transform, end_connection: Transform, opacity: float, valid: bool ):
        """
        Fill the FrameDrawingInfo object with the provided data.
        """
        obj = cls()
        obj.name = frame
        obj.transform = transform
        obj.start_connection = start_connection
        obj.end_connection = end_connection
        obj.opacity = opacity
        obj.valid = valid
        return obj
