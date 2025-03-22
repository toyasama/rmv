from dataclasses import dataclass
from typing import Tuple

@dataclass
class FramesPosition:
    name: str
    start: Tuple[int, int]
    end_x: Tuple[int, int]
    end_y: Tuple[int, int]
    opacity: float 
    start_connection: Tuple[int, int] = None
    end_connection: Tuple[int, int] = None

    