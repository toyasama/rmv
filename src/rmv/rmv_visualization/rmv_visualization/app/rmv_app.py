from kivy.app import App
from kivy.lang import Builder
from kivy.uix.boxlayout import BoxLayout
from kivy.clock import Clock
from kivy.graphics.texture import Texture
from pathlib import Path
import numpy as np
import cv2

from rmv_visualization.app.uix.control_panel import ControlPanel
from rmv_chore.rmv_chore_node import RMVChoreNode
from kivy.core.window import Window

Window.minimum_width, Window.minimum_height = 1200, 800

dir_path = Path(__file__).resolve().parent
kv_path = dir_path / "rmv_app.kv"

if not kv_path.exists():
    raise FileNotFoundError(f"Cannot find kv file : {kv_path}")

Builder.load_file(str(kv_path))



class Rmv(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def updateImage(self, image: np.ndarray):
        if image is None:
            return
        frame = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        frame = np.flip(frame, 0)
        height, width, _ = frame.shape
        texture = Texture.create(size=(width, height))
        texture.blit_buffer(frame.tobytes(), colorfmt='rgb', bufferfmt='ubyte')
        self.ids.video_feed.texture = texture
        
class RmvApp(App):
    def __init__(self, rmv_node:RMVChoreNode, **kwargs):
        super().__init__(**kwargs)
        self.visualization = rmv_node.visualization
        self.rmv_params = rmv_node.parameters
        self.transform_graph = rmv_node.transform_graph
        self.rmv_ui = None  
        self.period = rmv_node.period

    def build(self):
        self.rmv_ui = Rmv()
        Clock.schedule_interval(self.updateUi, self.period)
        return self.rmv_ui

    def updateUi(self, dt):
        if self.rmv_ui:
            image = self.visualization.image
            if image is not None:
                self.rmv_ui.updateImage(image)

    def on_stop(self):
        Clock.unschedule(self.updateUi)