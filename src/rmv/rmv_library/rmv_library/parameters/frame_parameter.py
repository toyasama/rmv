from dataclasses import dataclass, field
from typing import Dict, Any, List
from threading import RLock


@dataclass
class FramesParameters:
    _main_frame: str = field(default="", init=False)
    _sub_frames: List[str]= field(default_factory=list, init=False)
    _show_sub_frames: bool = field(default=True, init=False)
    _show_axes: bool = field(default=True, init=False)
    _show_frame_names: bool = field(default=True, init=False)
    _show_connections: bool = field(default=True, init=False)
    _axes_length: float = field(default=0.1, init=False)
    
    def __dict__(self) -> Dict[str, Any]:
        return {"frames":{
            'main_frame': self._main_frame,
            'sub_frames': self._sub_frames,
            'show_axes': self._show_axes,
            'show_frame_names': self._show_frame_names,
            'show_connections': self._show_connections,
            'axes_length': self._axes_length,
            'show_sub_frames': self._show_sub_frames
        }}
    
    def toDict(self) -> Dict[str, Any]:
        return self.__dict__()

    def __post_init__(self):
        self._locks = {
            'main_frame': RLock(),
            'sub_frames': RLock(),
            'show_axes': RLock(),
            'show_frame_names': RLock(),
            'show_connections': RLock(),
            'axes_length': RLock(),
            'show_sub_frames': RLock()
        }
    @property
    def show_sub_frames(self) -> bool:
        with self._locks['show_sub_frames']:
            return self._show_sub_frames
        
    @property
    def main_frame(self) -> str:
        with self._locks['main_frame']:
            return self._main_frame

    @main_frame.setter
    def main_frame(self, value: str) -> None:
        with self._locks['main_frame']:
            self._main_frame = value

    @property
    def sub_frames(self) -> Dict[str, Dict[str, Any]]:
        with self._locks['sub_frames']:
            return self._sub_frames.copy()
        
    def addSubFrame(self, frame_name: str) -> None:
        with self._locks['sub_frames']:
            if frame_name not in self._sub_frames:
                print("Adding sub frame ", frame_name)
                self._sub_frames.append(frame_name)
    
    def removeSubFrame(self, frame_name: str) -> None:
        with self._locks['sub_frames']:
            if frame_name in self._sub_frames:
                self._sub_frames.remove(frame_name)

    def updateSubFrame(self, frame_list:List) -> None:
        with self._locks['sub_frames']:
            self._sub_frames = frame_list

    @property
    def show_axes(self) -> bool:
        with self._locks['show_axes']:
            return self._show_axes

    def toggleAxes(self) -> None:
        with self._locks['show_axes']:
            self._show_axes = not self._show_axes

    @property
    def show_frame_names(self) -> bool:
        with self._locks['show_frame_names']:
            return self._show_frame_names

    def toggleFrameNames(self) -> None:
        with self._locks['show_frame_names']:
            self._show_frame_names = not self._show_frame_names

    @property
    def show_connections(self) -> bool:
        with self._locks['show_connections']:
            return self._show_connections

    def toggleConnections(self) -> None:
        with self._locks['show_connections']:
            self._show_connections = not self._show_connections
    
    def toggleSubFrames(self) -> None:
        with self._locks['show_sub_frames']:
            self._show_sub_frames = not self._show_sub_frames

    @property
    def axes_length(self) -> float:
        with self._locks['axes_length']:
            return self._axes_length

    @axes_length.setter
    def axes_length(self, value: float) -> None:
        if value > 0:
            with self._locks['axes_length']:
                self._axes_length = value
