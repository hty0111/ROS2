"""
 @Description: 
 @version: v1.0
 @Author: HTY
 @Date: 2022-07-23 15:52:53
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from learning_interface.action import Fibonacci


class FibonacciServer(Node):
    def __init__(self, node):
        super(FibonacciServer, self).__init__(node)
        self.action_server = ActionServer(self, Fibonacci, "fibonacci", self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order - 1):
            feedback_msg.partial_sequence.append(feedback_msg.partial_sequence[i-1] + feedback_msg.partial_sequence[i])
            self.get_logger().info(f"Feedback: {feedback_msg.partial_sequence}")
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result


def main(args=None):
    rclpy.init(args=args)
    fibonacci_server = FibonacciServer("fibonacci_server_node")
    rclpy.spin(fibonacci_server)


if __name__ == "__main__":
    main()

