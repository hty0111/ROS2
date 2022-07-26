"""
 @Description: 
 @version: v1.0
 @Author: HTY
 @Date: 2022-07-26 15:24:21
"""

import rclpy
from rclpy.node import Node
import tf_transformations
from tf2_ros import TransformBroadcaster

from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped


class FrameBroadcaster(Node):
    def __init__(self, node):
        super(FrameBroadcaster, self).__init__(node)

        self.declare_parameter('turtlename', 'turtle1')
        self.turtlename = self.get_parameter('turtlename').get_parameter_value().string_value
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(Pose, f'/{self.turtlename}/pose', self.turtle_pose_callback, 10)

    def turtle_pose_callback(self, msg):
        trans = TransformStamped()

        trans.header.stamp = self.get_clock().now().to_msg()
        trans.header.frame_id = 'world'
        trans.child_frame_id = self.turtlename

        trans.transform.translation.x = msg.x
        trans.transform.translation.y = msg.y
        trans.transform.translation.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
        trans.transform.rotation.x = q[0]
        trans.transform.rotation.y = q[1]
        trans.transform.rotation.z = q[2]
        trans.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(trans)


def main(args=None):
    rclpy.init(args=args)
    broadcaster = FrameBroadcaster('tf2_broadcaster_node')
    rclpy.spin(broadcaster)
    broadcaster.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
