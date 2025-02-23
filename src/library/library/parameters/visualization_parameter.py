from dataclasses import dataclass, field
from typing import Dict, Any, Tuple, Optional
from threading import RLock

@dataclass
class VisualizationParameters:
    _width: int = field(default=100, init=False)
    _height: int = field(default=100, init=False)
    _fps: int = field(default=30, init=False)
    _draw_grid: bool = field(default=True, init=False)
    _fov_deg: int = field(default=60, init=False)
    _background_color: Dict[str, int] = field(default_factory=lambda: {'r': 0, 'g': 0, 'b': 0}, init=False)
    _grid_color: Dict[str, int] = field(default_factory=lambda: {'r': 255, 'g': 255, 'b': 255}, init=False)
    _camera: Dict[str, Dict[str, Any]] = field(
        default_factory=lambda: {'position': {'x': 0, 'y': 0, 'z': 0, 'theta': 0}, 'fov_deg': 60}, init=False
    )
    _grid_spacing: float = field(default=1, init=False)
    
    def __post_init__(self):
        self._locks = {
            'width': RLock(),
            'height': RLock(),
            'fps': RLock(),
            'draw_grid': RLock(),
            'background_color': RLock(),
            'grid_color': RLock(),
            'camera': RLock(),
            'grid_spacing': RLock(),
            'fov': RLock()
        }
    @property
    def resolution(self) -> Tuple[int, int, int]:
        with self._locks['width'], self._locks['height'], self._locks['fov']:
            return self._width, self._height, self._fov_deg
    
    @property
    def width(self) -> int:
        with self._locks['width']:
            return self._width

    @width.setter
    def width(self, value: int) -> None:
        if value > 0:
            with self._locks['width']:
                self._width = value
    
    @property
    def height(self) -> int:
        with self._locks['height']:
            return self._height

    @height.setter
    def height(self, value: int) -> None:
        if value > 0:
            with self._locks['height']:
                self._height = value
    
    @property
    def fps(self) -> int:
        with self._locks['fps']:
            return self._fps

    @fps.setter
    def fps(self, value: int) -> None:
        if value > 0:
            with self._locks['fps']:
                self._fps = value
    
    @property
    def background_color(self) -> Dict[str, int]:
        with self._locks['background_color']:
            return self._background_color.copy()

    def updateBackgroundColor(self, r: int, g: int, b: int) -> None:
        with self._locks['background_color']:
            self._background_color = {'r': r, 'g': g, 'b': b}
    
    @property
    def camera_position(self) -> Tuple[float, float, float, float]:
        with self._locks['camera']:
            return tuple(self._camera['position'][key] for key in ['x', 'y', 'z', 'theta'])

    def updateCameraPosition(self, x: Optional[float] = None, y: Optional[float] = None,
                               z: Optional[float] = None, theta: Optional[float] = None) -> None:
        with self._locks['camera']:
            if x is not None:
                self._camera['position']['x'] = x
            if y is not None:
                self._camera['position']['y'] = y
            if z is not None:
                self._camera['position']['z'] = z
            if theta is not None:
                self._camera['position']['theta'] = theta
    
    @property
    def fov(self) -> int:
        with self._locks['camera']:
            return self._camera['fov_deg']

    @fov.setter
    def fov(self, value: int) -> None:
        if 10 <= value <= 180:
            with self._locks['camera']:
                self._camera['fov_deg'] = value
    
    @property
    def grid_spacing(self) -> float:
        with self._locks['grid_spacing']:
            return self._grid_spacing
    
    @grid_spacing.setter
    def grid_spacing(self, value: float) -> None:
        if value > 0:
            with self._locks['grid_spacing']:
                self._grid_spacing = value
                
    @property
    def draw_grid(self) -> bool:
        with self._locks['draw_grid']:
            return self._draw_grid
        
    @draw_grid.setter
    def draw_grid(self, value: bool) -> None:
        with self._locks['draw_grid']:
            self._draw_grid = value
    
    @property
    def grid_color(self) -> Dict[str, int]:
        with self._locks['grid_color']:
            return self._grid_color.copy()
    
    def updateGridColor(self, r:Optional[int] = None, g:Optional[int] = None,b:Optional[int] = None ) -> None:
        with self._locks['grid_color']:
            if r is not None:
                self._grid_color['r'] = r
            if g is not None:
                self._grid_color['g'] = g
            if b is not None:
                self._grid_color['b'] = b