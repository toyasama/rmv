import rclpy
from rmv_chore.rmv_chore import RmvChore
def main():
    rclpy.init()
    rmv_chore = RmvChore()
    try:
        rclpy.spin(rmv_chore.getNode())
    except KeyboardInterrupt:
        pass
    finally:
        rmv_chore.destroyNode()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()