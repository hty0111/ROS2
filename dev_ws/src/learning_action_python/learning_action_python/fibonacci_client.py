"""
 @Description: 
 @version: v1.0
 @Author: HTY
 @Date: 2022-07-23 16:20:53
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from learning_interface.action import Fibonacci


class FibonacciClient(Node):
    def __init__(self, node):
        super(FibonacciClient, self).__init__(node)
        self.action_client = ActionClient(self, Fibonacci, "fibonacci")
        self.send_future = None
        self.result_future = None

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for server started...")
        self.send_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected!")
            return

        self.get_logger().info("Goal accepted!")
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.sequence}")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback.partial_sequence}")


def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciClient("fibonacci_client_node")
    action_client.send_goal(10)
    rclpy.spin(action_client)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
