from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
        
from kivy.lang import Builder
import os
from pathlib import Path


dir_path = Path(__file__).resolve().parent
path_to_kv = os.path.join(dir_path, 'control_panel.kv')
Builder.load_file(path_to_kv)
        
class ControlPanel(BoxLayout):
    def __init__(self,  **kwargs):
        super().__init__(**kwargs)
        self.app = App.get_running_app()
        
    def toggle_draw_grid(self):
        self.app.rmv_params.visualization.toggleDrawGrid()

    def toggle_show_axes(self):
        self.app.rmv_params.frames.toggleAxes()
