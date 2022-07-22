"""
 @Description: ROS2 topic接收自定义消息 python版本
 @version: v1.0
 @Author: HTY
 @Date: 2022-07-20 18:39:24
"""

import rclpy
from rclpy.node import Node
from learning_interface.msg import PersonInfo


class PersonInfoSub(Node):
    def __init__(self, node):
        super().__init__(node)
        self.subscription = self.create_subscription(PersonInfo, "person_info_topic", self.person_info_callback, 10)
        
    def person_info_callback(self, msg):
        self.get_logger().info(f"name: {msg.name}, age: {msg.age}, id: {msg.id}")
        

def main(args=None):
    rclpy.init(args=args)
    person_info_node = PersonInfoSub("person_info_sub_node")
    rclpy.spin(person_info_node)
    person_info_node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
    