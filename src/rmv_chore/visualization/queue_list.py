import queue
from typing import List
from markers_management.markers import MarkerRmv

class QueueList:
    """
    A queue that holds a list of items.
    """
    def __init__(self):
        self.markers_queue = queue.Queue()
        self.main_tf_queue = queue.Queue()
        
    def putMarkers(self, markers: List[MarkerRmv])->None:
        """ 
        Put a list of markers in the queue.
        Args:
            markers (List[MarkerRmv]): A list of markers.
        """
        self.markers_queue.put(markers)
    
    def putMainTf(self, tf: str)->None:
        """
        Put a TF frame in the queue.
        Args:
            tf (str): The TF frame name.
        """
        self.main_tf_queue.put(tf)

    def getMarkers(self) -> List[MarkerRmv]:
        """
        Get a list of markers from the queue.
        Returns:
            List[MarkerRmv]: A list of markers.
        """
        return self.markers_queue.get()
    
    def getMainTf(self) -> str:
        """
        Get a TF frame from the queue.
        Returns:
            str: The TF frame name.
        """
        return self.main_tf_queue.get()
    
    def markersEmpty(self) -> bool:
        """
        Check if the markers queue is empty.
        Returns:
            bool: True if the markers queue is empty, False otherwise.
        """
        return self.markers_queue.empty()
    
    def mainTfEmpty(self) -> bool:
        """
        Check if the main TF queue is empty.
        Returns:
            bool: True if the main TF queue is empty, False otherwise.
        """
        return self.main_tf_queue.empty()
    