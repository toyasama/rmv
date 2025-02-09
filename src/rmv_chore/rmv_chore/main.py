import rclpy
from library import dataManager, VisualizationParams, TimerLogger
from visualization.visualization import Visualization
from std_msgs.msg import ColorRGBA
from threading import Thread
from time import sleep

def create_rmv_chore_node():
    """
    Function to create and initialize the rmv_chore node and related components.
    """
    rclpy.init()

    node = rclpy.create_node("rmv_chore", namespace="rmv")
    
    background_color = ColorRGBA(r=0.1, g=0.1, b=0.1, a=1.0)
    visu_params = VisualizationParams(width=800, height=600, fps=30, background_color=background_color)
    
    visualization = Visualization(node, visu_params)
    timer_logger = TimerLogger(node, 2.0)
    timer_logger2 = TimerLogger(node, 2.0)
    timer_logger_loop = TimerLogger(node, 2.0)
    data_manager = dataManager(node)
    
    _is_running = True

    def update_markers():
        """
        Method to update the markers list and push filtered markers to the queue.
        """
        while _is_running:
            timer_logger_loop.logExecutionTime(loop)()
            sleep(0.1)
    
    def loop():
        """
        Method to run the node.
        """
        timer_logger2.logExecutionTime(data_manager.run)()
        timer_logger.logExecutionTime(visualization.visualize)(data_manager.shared_data)

    thread = Thread(target=update_markers, daemon=True)
    thread.start()

    

def main():
    rclpy.init()

    node = rclpy.create_node("rmv_chore", namespace="rmv")
    
    background_color = ColorRGBA(r=0.1, g=0.1, b=0.1, a=1.0)
    visu_params = VisualizationParams(width=800, height=600, fps=30, background_color=background_color)
    
    visualization = Visualization(node, visu_params)
    timer_logger = TimerLogger(node, 2.0)
    timer_logger2 = TimerLogger(node, 2.0)
    timer_logger_loop = TimerLogger(node, 2.0)
    data_manager = dataManager(node)

    def update_markers():
        """
        Method to update the markers list and push filtered markers to the queue.
        """
        while True:
            try:
                timer_logger_loop.logExecutionTime(loop)()
                if not rclpy.ok():
                    break
                rclpy.spin_once(node)
                
            except KeyboardInterrupt:
                break
            sleep(0.01)
        rclpy.shutdown()
    
    def loop():
        """
        Method to run the node.
        """
        timer_logger2.logExecutionTime(data_manager.processData)()
        timer_logger.logExecutionTime(visualization.visualize)(data_manager.shared_data)

    thread = Thread(target=update_markers, daemon=True)
    thread.start()


    
if __name__ == "__main__":
    main()