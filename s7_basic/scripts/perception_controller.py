#!/usr/bin/env python3
import rclpy
from asl_tb3_lib.control import BaseController
from asl_tb3_msgs.msg import TurtleBotControl

class PerceptionController(BaseController):
    def __init__(self):
        super().__init__("perception_controller")
        self.declare_parameter("active", True)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
    
    @property
    def active(self) -> bool:
        """ Get real-time parameter value of active status

        Returns:
            bool: latest parameter value of active status
        """
        return self.get_parameter("active").value
    
    def compute_control(self) -> TurtleBotControl:
        command = TurtleBotControl()
        if self.active:
            command.omega = 0.5
            self.start_time = self.get_clock().now().nanoseconds / 1e9

        else:
            cur_time = self.get_clock().now().nanoseconds / 1e9
            if cur_time - self.start_time <= 5.0:
                command.omega = 0.0
            else:
                self.start_time = cur_time
                self.set_parameters([rclpy.Parameter("active", value=True)])
            
        return command

if __name__ == "__main__":
    rclpy.init()        # initialize ROS2 context (must run before any other rclpy call)
    node = PerceptionController()  # instantiate the heading controller node
    rclpy.spin(node)    # Use ROS2 built-in schedular for executing the node
    rclpy.shutdown()    # cleanly shutdown ROS2 context