"""
 @Description: ROS2 topic发送自定义消息 python版本
 @version: v1.0
 @Author: HTY
 @Date: 2022-07-20 15:01:52
"""

import rclpy
from rclpy.node import Node
from learning_interface.msg import PersonInfo


class PersonInfoPub(Node):
    def __init__(self, node):
        super().__init__(node)
        self.publisher = self.create_publisher(PersonInfo, "person_info_topic", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.counter = 1
        
    def timer_callback(self):
        person_msg = PersonInfo()
        person_msg.name = "HTY"
        person_msg.age = 21
        person_msg.id = 3190105708
        self.publisher.publish(person_msg)
        self.get_logger().info(f"Loop: {self.counter} | name: {person_msg.name} | age: {person_msg.age} | id: {person_msg.id}")
        self.counter += 1
        

def main(args=None):
    rclpy.init(args=args)
    person_info_node = PersonInfoPub("person_info_pub_node")
    rclpy.spin(person_info_node)
    person_info_node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
