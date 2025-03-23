from kivy.uix.textinput import TextInput
from kivy.uix.label import Label
from kivy.core.window import Window
from kivy.lang import Builder
import os
from pathlib import Path


# dir_path = Path(__file__).resolve().parent
# path_to_kv = os.path.join(dir_path, 'custom.kv')
# Builder.load_file(path_to_kv)

class CustomInput(TextInput):
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.multiline = False
        self.halign = 'center'
        self.valign = 'center'


class CustomLabel(Label):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.color = [0, 0, 0, 1]
        self.valign = 'center'
        self.halign = 'left'
        self.markup = True
        self.text_size = (self.width, self.height)  
        self.bind(size=self.update_text_size) 
        Window.bind(on_resize=self.update_text_size)

    def update_text_size(self, *args):
        self.text_size = (self.width, self.height)  
        self.texture_update()  
        