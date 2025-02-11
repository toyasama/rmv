import time
from rclpy.node import Node


class TimerLogger:
    def __init__(self, node: Node, period: float = 0., enabled: bool = True):
        """
        Timer utility for logging execution time of methods.
        Args:
            node (Node): The ROS2 node object.
            period (float): Period for periodic logging in seconds.
            enabled (bool): Whether the timer is enabled.
        """
        self.node = node
        self.period = period
        self.enabled = False
        self.lastLogTime = time.monotonic()
        self.executionTimes = []

        node.get_logger().info(
            f"TimerLogger initialized with period={self.period}s and enabled={self.enabled}"
        )

    def _recordExecutionTime(self, funcName: str, elapsedTime: float) -> None:
        """
        Records execution time and logs periodically if needed.
        Args:
            funcName (str): The name of the function.
            elapsedTime (float): The execution time in milliseconds.
        """
        self.executionTimes.append(elapsedTime)
        currentTime = time.monotonic()

        if (currentTime - self.lastLogTime) >= self.period:
            meanExecutionTime = sum(self.executionTimes) / len(self.executionTimes)
            self.node.get_logger().info(
                f"Mean execution time for {funcName}: {meanExecutionTime:.2f} ms"
            )
            self.executionTimes = []
            self.lastLogTime = currentTime

    def logExecutionTime(self, func):
        """
        Decorator to measure and log the execution time of a method.
        Logs periodically if a period is defined.
        Args:
            func: The function to decorate.
        """
        def wrapper(*args, **kwargs):
            if not self.enabled:
                return func(*args, **kwargs)

            startTime = time.monotonic()
            result = func(*args, **kwargs)
            endTime = time.monotonic()

            elapsedTime = (endTime - startTime) * 1000  # Convert to ms
            if self.period > 0:
                self._recordExecutionTime(func.__name__, elapsedTime)
            else:
                self.node.get_logger().info(
                    f"Execution time for {func.__name__}: {elapsedTime:.2f} ms"
                )
            return result

        return wrapper
