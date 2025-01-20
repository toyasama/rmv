from std_msgs.msg import ColorRGBA

class VisualizationParams:
    """
    Parameters for the Visualization class.
    """
    def __init__(self, width: int, height: int, fps: int, background_color: ColorRGBA):
        self.width = width
        self.height = height
        self.fps = fps
        self.background_color = background_color
        self.conversion_ratio = 1.0 / 0.01  # 1 meter in real life equals 10 pixels in the image
        self.publish_image = True


class  RmvParams():
    """
    RMV Chore parameters

    Args:
        UPDATE_TOPIC_PROCESS_PERIOD (float): Period to check for markers topics in topic manager.
        UPDATE_MARKERS_PROCESS_PERIOD (float)
    """
    UPDATE_TOPIC_PROCESS_PERIOD = 0.25
    UPDATE_MARKERS_PROCESS_PERIOD = 0.1
    TIMEOUT_TF_BUFFER = 0.1
