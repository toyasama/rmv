import time
from rclpy.node import Node


class TimerLogger:
    def __init__(self, node: Node, enabled: bool = True):
        """
        Timer utility for logging execution time of methods.
        Args:
            node (Node): The ROS2 node object.
            enabled (bool): Whether the timer is enabled or not.
        """
        self.node = node
        self.enabled = enabled

    def logExecutionTime(self, func):
        """
        Decorator to measure and log the execution time of a method.
        Args:
            func: The function to decorate.
        """
        def wrapper(*args, **kwargs):
            if not self.enabled:
                return func(*args, **kwargs)

            start_time = time.monotonic()
            result = func(*args, **kwargs)
            end_time = time.monotonic()

            elapsed_time = (end_time - start_time) * 1000  # Convert to ms
            self.node.get_logger().info(f"Execution time for {func.__name__}: {elapsed_time:.2f} ms")
            return result
        return wrapper
