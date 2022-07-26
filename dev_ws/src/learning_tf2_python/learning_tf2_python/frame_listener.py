"""
 @Description: 
 @version: v1.0
 @Author: HTY
 @Date: 2022-07-26 16:02:25
"""

import math

import rclpy
from rclpy import time, duration
from rclpy.node import Node
from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist


class FrameListener(Node):
    def __init__(self, node):
        super(FrameListener, self).__init__(node)

        self.declare_parameter('source_frame', 'turtle1')
        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        self.declare_parameter('target_frame', 'turtle2')
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.spawner = self.create_client(Spawn, 'spawn')
        self.turtle_spawn_service_ready = False
        self.turtle_spawned = False
        self.result = None

        self.publisher = self.create_publisher(Twist, f'{self.target_frame}/cmd_vel', 10)

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        if self.turtle_spawn_service_ready:
            if self.turtle_spawned:
                try:
                    trans = self.tf_buffer.lookup_transform(
                        self.target_frame,
                        self.source_frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.05)
                    )
                except TransformException as ex:
                    self.get_logger().error(f'Could not transform {self.target_frame} to {self.source_frame}: {ex}')
                    return
                except (LookupException, ConnectivityException, ExtrapolationException) as ex:
                    self.get_logger().error(f'Transform not ready: {ex}')
                    return

                msg = Twist()
                scale_rotation_rate = 1.0
                msg.angular.z = scale_rotation_rate * math.atan2(trans.transform.translation.y,
                                                                 trans.transform.translation.x)

                scale_forward_rate = 0.5
                msg.linear.x = scale_forward_rate * math.hypot(trans.transform.translation.x,
                                                               trans.transform.translation.y)

                self.publisher.publish(msg)

            else:   # turtle spawned
                if self.result.done():
                    self.get_logger().info(f'Successfully spawned {self.result.result().name}')
                    self.turtle_spawned = True
                else:
                    self.get_logger().warn('Spawn is not finished')

        else:   # service ready
            if self.spawner.service_is_ready():
                request = Spawn.Request()
                request.name = self.target_frame
                request.x = float(4)
                request.y = float(2)
                request.theta = float(0)

                self.result = self.spawner.call_async(request)
                self.turtle_spawn_service_ready = True
            else:
                # Check if the service is ready
                self.get_logger().info('Service is not ready')


def main(args=None):
    rclpy.init(args=args)
    listener = FrameListener('tf2_listener_node')
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
