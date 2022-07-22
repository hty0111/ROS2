"""
 @Description: 
 @version: v1.0
 @Author: HTY
 @Date: 2022-07-22 18:05:11
"""

from learning_interface.srv import PersonInfo

import rclpy
from rclpy.node import Node


class PersonInfoServer(Node):
    def __init__(self, node):
        super(PersonInfoServer, self).__init__(node)
        self.server = self.create_service(PersonInfo, "person_info_service", self.person_callback)
        self.get_logger().info("Service started")

    def person_callback(self, request, response):
        if request.name.upper() == "HTY":
            response.age = 21
            response.id = 3190105708
        elif request.name.lower() == "ZXA":
            response.age = 20
            response.id = 3190105598
        response.message = f"Get person info for {request.name}!  age: {response.age}  id: {response.id}"

        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    person_info_server = PersonInfoServer("person_info_server_node")
    rclpy.spin(person_info_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
