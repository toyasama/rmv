import rclpy
from rmv_chore.rmv_chore import RmvChore
def main():
    rclpy.init()
    rmv_chore = RmvChore()
    rclpy.spin(rmv_chore.getNode())
    rmv_chore.destroyNode()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()