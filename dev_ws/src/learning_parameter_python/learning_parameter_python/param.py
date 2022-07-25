"""
 @Description: 
 @version: v1.0
 @Author: HTY
 @Date: 2022-07-25 00:55:27
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor


class Param(Node):
    def __init__(self, node):
        super(Param, self).__init__(node)
        self.timer = self.create_timer(2, self.timer_callback)
        param_description = ParameterDescriptor(description="This param is for learning.")
        self.declare_parameter('learn_param', 'world', param_description)

    def timer_callback(self):
        param = self.get_parameter('learn_param').get_parameter_value().string_value
        self.get_logger().info("Hello %s!" % param)
        new_param = rclpy.parameter.Parameter('learn_param', rclpy.Parameter.Type.STRING, param)
        all_new_params = [new_param]
        self.set_parameters(all_new_params)


def main(args=None):
    rclpy.init(args=args)
    param_node = Param("param_node")
    rclpy.spin(param_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
