import rclpy
from threading import Thread, RLock
from time import sleep, time
from library import dataManager, VisualizationParams, TimerLogger
from visualization.visualization import Visualization
from std_msgs.msg import ColorRGBA
from library.utils.shared_data import SharedData
def main():
    rclpy.init()
    node = rclpy.create_node("rmv_chore", namespace="rmv")
    
    background_color = ColorRGBA(r=0.1, g=0.1, b=0.1, a=1.0)
    visu_params = VisualizationParams(width=800, height=600, fps=30, background_color=background_color)
    shared_data = SharedData()
    visualization = Visualization(node, visu_params, shared_data)
    data_manager = dataManager(node, shared_data)
    
    running = True
    
    def update_markers():
        """
        Met à jour la liste des marqueurs et les met en file d'attente après filtrage.
        """
        while running:
            rclpy.spin_once(node, timeout_sec=0.01)
            sleep(0.01)
    
    def processData():
        """
        Fonction exécutée en boucle pour traiter les données et les visualiser.
        """
        while running:
            data_manager.processData()
            sleep(0.01)
    
    def visualize():
        while running:
            visualization.visualize()
            sleep(0.01)
            
    spinner_thread = Thread(target=update_markers, daemon=True)
    data_thread = Thread(target=processData, daemon=True)
    visu_thread = Thread(target=visualize, daemon=True)
    
    spinner_thread.start()
    data_thread.start()
    visu_thread.start()
    
    try:
        spinner_thread.join()
        data_thread.join()
        visu_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        rclpy.shutdown()

if __name__ == "__main__":
    main()
