import yaml
import os
from dataclasses import dataclass
from typing import Dict

@dataclass
class VisualizationParameters:
    width: int = 100
    height: int = 100
    fps: int = 30
    draw_grid: bool = True
    background_color: Dict[str, int] = None
    grid_color: Dict[str, int] = None
    camera: Dict[str, Dict[str, int]] = None

    def __post_init__(self):
        if self.background_color is None:
            self.background_color = {'r': 0, 'g': 0, 'b': 0}
        if self.grid_color is None:
            self.grid_color = {'r': 255, 'g': 255, 'b': 255}
        if self.camera is None:
            self.camera = {'position': {'x': 0, 'y': 0, 'z': 0, 'theta':0}, 'fov_deg': 60}

    @property
    def resolution(self):
        return self.width, self.height, self.camera['fov_deg']

    @resolution.setter
    def resolution(self, value):
        self.width, self.height = value
        
    @property
    def camera_position(self):
        return [self.camera['position'][key] for key in ['x', 'y', 'z', 'theta']]
    
class RmvParameters:
    def __init__(self, path: str):
        self.__path = path
        self.data = self._loadYaml()
        print(self.data)
        self.visualization = VisualizationParameters(**self.data.get('RMV', {}).get('visualizations', {}))

    def _loadYaml(self) -> Dict:
        """
        Load the YAML file and apply default values if necessary.
        Returns:
            The loaded data as a dictionary."""
        if not os.path.exists(self.__path):
            print(f"File {self.__path} not found, creating an empty file.")
            return {}
        
        try:
            with open(self.__path, 'r', encoding='utf-8') as file:
                data = yaml.safe_load(file) or {}
        except Exception as e:
            print(f"Error reading the file: {e}")
            return {}
        
        return self._applyDefaults(data)

    def _applyDefaults(self, data: Dict) -> Dict:
        """Apply default values if some keys are missing.
        Args:
            data: The loaded data as a dictionary.
        Returns:
            The updated data as a dictionary."""
        defaults = {
            'RMV': {
                'visualizations': VisualizationParameters().__dict__
            }
        }
        return self._mergeDefaults(defaults, data)

    def _mergeDefaults(self, defaults: Dict, data: Dict) -> Dict:
        """
        Merge default values with those loaded from the file.
        Args:
            defaults: The default values as a dictionary.
            data: The loaded data as a dictionary.
        Returns:
            The updated data as a dictionary.
        """
        if isinstance(defaults, dict) and isinstance(data, dict):
            for key, value in defaults.items():
                if key not in data:
                    print(f"Missing key: {key}, assigning default value.")
                    data[key] = value
                else:
                    data[key] = self._mergeDefaults(value, data[key])
        return data

    def get(self, *keys):
        """Retrieve a value from the given keys."""
        value = self.data
        for key in keys:
            if key in value:
                value = value[key]
            else:
                print(f"Key {'.'.join(keys)} not found, returning default value.")
                return None
        return value

    def set(self, value, *keys):
        """Update a value and save it to the YAML file."""
        d = self.data
        for key in keys[:-1]:
            if key not in d:
                d[key] = {}
            d = d[key]
        d[keys[-1]] = value
        self._saveYaml()

    def _saveYaml(self):
        """Save the updated data to the YAML file."""
        try:
            with open(self.__path, 'w', encoding='utf-8') as file:
                yaml.dump(self.data, file, default_flow_style=False, allow_unicode=True)
        except Exception as e:
            print(f"Error writing to the file: {e}")
