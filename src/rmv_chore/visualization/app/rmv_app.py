from kivy.app import App
from kivy.lang import Builder
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.textinput import TextInput
from kivy.clock import Clock
from kivy.graphics.texture import Texture
from pathlib import Path
from time import time
import numpy as np
import cv2

from visualization.visualization import Visualization
from library import RmvParameters
from kivy.core.window import Window
from kivy.properties import StringProperty, ObjectProperty, NumericProperty, BooleanProperty

Window.minimum_width, Window.minimum_height = 1200, 800

dir_path = Path(__file__).resolve().parent
kv_path = dir_path / "rmv_app.kv"

if not kv_path.exists():
    raise FileNotFoundError(f"Cannot find kv file : {kv_path}")

Builder.load_file(str(kv_path))

class ParameterInput(BoxLayout ):
    label_text = StringProperty("")
    hint_text = StringProperty("")
    input_text = StringProperty("")
    input_clbk = ObjectProperty(None)
    
    def callback(self, value):
        if self.input_clbk:
            self.input_clbk(value)

class ColorInput(TextInput):
    last_valid_input = StringProperty('')
    
    def insert_text(self, substring, from_undo=False):
        new_text = self.text + substring
        
        if new_text.isdigit() and 0 <= int(new_text) <= 255:
            super().insert_text(substring, from_undo)
            self.last_valid_input = new_text
        elif new_text == '':
            super().insert_text(substring, from_undo)
            self.last_valid_input = ''
        else:
            pass
        
class ImageSizeParameter(BoxLayout):
    """
    Widget réutilisable pour un paramètre de taille d'image.
    """
    label_text = StringProperty("")
    im_width = StringProperty("0")
    im_height = StringProperty("0")
    resolution_change_clbk = ObjectProperty(None)
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
    
    def _widthClbk(self, value):
        self.im_width = value
        value =  int(value)
        if self.resolution_change_clbk:
            self.resolution_change_clbk(width = value)
        
    def _heightClbk(self, value):
        self.im_height = value
        value =  int(value)
        if self.resolution_change_clbk:
            self.resolution_change_clbk(height = value)
        
class ColorParameter(BoxLayout):
    """
    Widget réutilisable pour un paramètre de couleur.
    """
    label_text = StringProperty("")
    r_color = StringProperty("0")
    g_color = StringProperty("0")
    b_color = StringProperty("0")
    color_change_clbk = ObjectProperty(None)
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
    
    def _rColorClbk(self, value):
        self.r_color = value
        value =  int(value)
        if self.color_change_clbk:
            self.color_change_clbk(r=value)
        
    def _gColorClbk(self, value):
        self.g_color = value
        value =  int(value)
        if self.color_change_clbk:
            print(self.g_color)
            self.color_change_clbk(g= value)
        
    def _bColorClbk(self, value):
        self.b_color = value
        value =  int(value)
        if self.color_change_clbk:
            self.color_change_clbk(b=value)

class ParameterToggle(BoxLayout):
    """
    Widget réutilisable pour un paramètre avec un label et une checkbox.
    """
    label_text = StringProperty("")
    init_state = BooleanProperty(False)
    toggle_func = ObjectProperty(None)
    height = NumericProperty(0.1)  

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def on_toggle(self, instance, value):
        if self.toggle_func:
            self.toggle_func()
            
class MainFrameParameter(BoxLayout):
    height = NumericProperty(0.1)
    main_frame_name = StringProperty()
     
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
    
    def callback(self, value):
        app = App.get_running_app()
        if value and value != self.main_frame_name:
            print(f"main frame changed to {value}")
            app.rmv_params.frames.main_frame = value
            self.main_frame = value
        
        
class ControlPanel(BoxLayout):
    def __init__(self,  **kwargs):
        super().__init__(**kwargs)
        self.app = App.get_running_app()
        

    def toggle_draw_grid(self):
        self.app.rmv_params.visualization.toggleDrawGrid()

    def toggle_show_axes(self):
        self.app.rmv_params.frames.toggleAxes()

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
    def __init__(self, visualization: Visualization, rmv_params: RmvParameters, **kwargs):
        super().__init__(**kwargs)
        self.visualization = visualization
        self.rmv_params = rmv_params
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