"""
 @Description: 
 @version: v1.0
 @Author: HTY
 @Date: 2022-07-26 20:18:06
"""

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped


class FixedFrameBroadcaster(Node):
    def __init__(self, node):
        super(FixedFrameBroadcaster, self).__init__(node)

        self.fixed_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        trans = TransformStamped()
        trans.header.stamp = self.get_clock().now().to_msg()
        trans.header.frame_id = 'turtle1'
        trans.child_frame_id = 'carrot1'
        trans.transform.translation.x = 0.0
        trans.transform.translation.y = 2.0
        trans.transform.translation.z = 0.0
        trans.transform.rotation.x = 0.0
        trans.transform.rotation.y = 0.0
        trans.transform.rotation.z = 0.0
        trans.transform.rotation.w = 1.0
        self.fixed_broadcaster.sendTransform(trans)


def main(args=None):
    rclpy.init(args=args)
    fixed_broadcaster = FixedFrameBroadcaster('fixed_broadcaster_node')
    rclpy.spin(fixed_broadcaster)
    fixed_broadcaster.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
