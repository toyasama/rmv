from kivy.app import App
from kivy.lang import Builder
from kivy.uix.boxlayout import BoxLayout
from kivy.clock import Clock
from kivy.graphics.texture import Texture
from pathlib import Path
from time import time
import numpy as np
import cv2
from visualization.app.uix import CustomInput, CustomLabel
from visualization.app.uix.parameter import ParameterInput, ParameterToggle, MultipleParameterInput, ImageSizeParameter
from visualization.app.uix.control_panel import ControlPanel

from visualization.visualization import Visualization
from library import (RmvParameters, TransformGraph)
from kivy.core.window import Window
from kivy.properties import StringProperty, ObjectProperty, NumericProperty, BooleanProperty

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
    def __init__(self, visualization: Visualization, rmv_params: RmvParameters,transform_graph:TransformGraph, **kwargs):
        super().__init__(**kwargs)
        self.visualization = visualization
        self.rmv_params = rmv_params
        self.transform_graph = transform_graph
        self.rmv_ui = None  

    def build(self):
        self.rmv_ui = Rmv()
        Clock.schedule_interval(self.updateUi, 1 / 30)
        return self.rmv_ui

    def updateUi(self, dt):
        if self.rmv_ui:
            image = self.visualization.image
            if image is not None:
                self.rmv_ui.updateImage(image)

    def on_stop(self):
        Clock.unschedule(self.updateUi)