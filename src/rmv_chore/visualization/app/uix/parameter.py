from kivy.uix.boxlayout import BoxLayout
from kivy.properties import StringProperty, ObjectProperty, NumericProperty, BooleanProperty
from kivy.app import App
from library import ( FramesParameters)
from kivy.clock import Clock

from .custom import CustomInput

from kivy.lang import Builder
import os
from pathlib import Path

dir_path = Path(__file__).resolve().parent
path_to_kv = os.path.join(dir_path, 'parameter.kv')
Builder.load_file(path_to_kv)


            
class ColorInputParameter(CustomInput):
    last_valid_input = StringProperty('')
    
    def input_filter(self, text:str, from_undo=False):
        if not text.isdigit():
            return ''  
        
        new_text :str = self.text + text
        if new_text.isdigit() and 0 <= int(new_text) <= 255:
            return text  
        else:
            return ''
    
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

class ParameterToggle(BoxLayout):
    """
    Widget réutilisable pour un paramètre avec un label et une checkbox.
    """
    label_text = StringProperty("")
    init_state = BooleanProperty(False)
    disabled_state = BooleanProperty(False)
    toggle_func = ObjectProperty(None)
    height = NumericProperty(0.1)  

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def on_toggle(self, instance, value):
        if self.toggle_func:
            self.toggle_func()
            
    def set_disabled(self, state):
        self.disabled_state = state
        self.ids.toggle_checkbox.disabled = state
        
        
class ParameterInput(BoxLayout ):
    label_text = StringProperty("")
    hint_text = StringProperty("")
    input_text = StringProperty("")
    input_clbk = ObjectProperty(None)
    filter_clbk = ObjectProperty(None)
    
    def callback(self, value):
        if self.input_clbk:
            self.input_clbk(value)

class MultipleParameterInput(BoxLayout ):
    label_text = StringProperty("")
    hint_text = StringProperty("")
    input_text = StringProperty("")
    input_clbk = ObjectProperty(None)
    filter_clbk = ObjectProperty(None)
    
    def callback(self, value):
        if self.input_clbk:
            self.input_clbk(value)
            
            
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
            
            
            
        

class SubFramesList(BoxLayout):
    def __init__(self,  **kwargs):
        super().__init__(**kwargs)
        self.orientation = 'vertical'
        self.size_hint_y = None
        self.frames_params:FramesParameters = App.get_running_app().rmv_params.frames
        self.sub_frames_list = App.get_running_app().transform_graph.sub_frames
        widget_number =  len(self.sub_frames_list) + 1
        widget_height = self.height / widget_number
        self.height = widget_number * widget_height 
        self.param_toggles = [] 

        self.addAllParams(widget_height)
        self.addSubFrame(widget_height)

    def addAllParams(self, widget_height):
        self.show_all_frames_widget = ParameterToggle(label_text="Show all", height=widget_height)
        self.show_all_frames_widget.init_state = self.frames_params.show_sub_frames
        self.show_all_frames_widget.toggle_func = self.toggle_all_params 
        self.add_widget(self.show_all_frames_widget)

    def addSubFrame(self, widget_height):
        
        for sub_frame in self.sub_frames_list:
            param_toggle = ParameterToggle(label_text=sub_frame, height=widget_height)
            param_toggle.ids.toggle_checkbox.bind(active=lambda instance, value, sub_frame=sub_frame: self.updateSubFrameList(sub_frame, value))
            param_toggle.ids.toggle_checkbox.active = sub_frame in self.frames_params.sub_frames
            self.param_toggles.append(param_toggle)
            self.add_widget(param_toggle)
        self.__updateDisableState()
        
    def updateSubFrameList(self,sub_frame_name, value):
        print(f"update sub frame list {sub_frame_name}")
        if value:
            self.frames_params.addSubFrame(sub_frame_name)
        else:
            self.frames_params.removeSubFrame(sub_frame_name)
        

    def toggle_all_params(self):
        """Active ou désactive tous les sous-paramètres en fonction de 'Show all'."""
        self.frames_params.toggleSubFrames()
        self.__updateDisableState()
            
    def __updateDisableState(self):
        state = self.show_all_frames_widget.ids.toggle_checkbox.active
        for param_toggle in self.param_toggles:
            param_toggle.set_disabled(state)  


class SubFramesParameter(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.is_extended = False
        self.sub_frames_list_widget = None
        self.initial_height = self.height
        self.updating_size = False  

    def on_size(self, instance, value):
        """Adapte immédiatement la taille du widget pour réduire le lag visuel."""
        if self.updating_size:
            return 
        self.recalculate_height()

    def recalculate_height(self):
        """Met à jour la hauteur de manière optimisée."""
        
        if self.sub_frames_list_widget:
            new_height = (len(self.sub_frames_list_widget.sub_frames_list) + 1) * self.initial_height
            updated_height = self.initial_height + new_height
            if abs(self.height - updated_height) > 1: 
                self.updating_size = True
                self.height = updated_height
                Clock.unschedule(self.reset_updating_flag)  # Make sure to reset the flag only once and delete the previous scheduled reset
                Clock.schedule_once(self.reset_updating_flag, -1)  

    def reset_updating_flag(self, dt):
        self.updating_size = False
        
    def _toggleSubFrames(self):
        frames :FramesParameters= App.get_running_app().rmv_params.frames
        frames.toggleSubFrames()

    def toggleExtension(self):
        if not self.is_extended:
            self.extendWidget()
        else:
            self.collapseWidget()
        self.is_extended = not self.is_extended

    def extendWidget(self):
        self.sub_frames_list = App.get_running_app().transform_graph.sub_frames
        if not self.sub_frames_list_widget and len(self.sub_frames_list) > 0: 
            self.height = self.initial_height
            self.sub_frames_list_widget = SubFramesList( height=(len(self.sub_frames_list)+1)*self.height)
            self.add_widget(self.sub_frames_list_widget)
            self.height += self.sub_frames_list_widget.height
            
            

    def collapseWidget(self):
        if self.sub_frames_list_widget: 
            self.remove_widget(self.sub_frames_list_widget)
            self.sub_frames_list_widget = None  
            self.height = self.initial_height

