"""
 @Description: 
 @version: v1.0
 @Author: HTY
 @Date: 2022-07-22 17:17:59
"""

from learning_interface.srv import PersonInfo
import sys
import rclpy
from rclpy.node import Node


class PersonInfoClient(Node):
    def __init__(self, node):
        super(PersonInfoClient, self).__init__(node)
        self.client = self.create_client(PersonInfo, "person_info_service")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")
        self.req = PersonInfo.Request()
        self.future = None

    def send_person_info(self):
        self.req.name = str(sys.argv[1])
        self.future = self.client.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    person_info_client = PersonInfoClient("person_info_client_node")
    person_info_client.send_person_info()

    while rclpy.ok():
        rclpy.spin_once(person_info_client)
        if person_info_client.future.done():
            try:
                response = person_info_client.future.result()
            except Exception as e:
                person_info_client.get_logger().warn("Service call failed: %s" % e)
            else:
                person_info_client.get_logger().info(f"{response.message}")
            break

    person_info_client.destroy_node()
    rclpy.shutdown()

        
if __name__ == "__main__":
    main()
