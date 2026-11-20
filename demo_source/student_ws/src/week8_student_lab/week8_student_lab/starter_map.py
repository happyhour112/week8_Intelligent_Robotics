#!/usr/bin/env python3
"""Stage 1: small FIRST ROS 2 node, suitable for students to retype.

It publishes an occupancy grid at 2 Hz. There is no planner here.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from .world import W, H, RES, WALLS


class MapTeacher(Node):
    def __init__(self):
        super().__init__('week8_map_teacher')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(OccupancyGrid,'/student/map',latched)
        self.timer = self.create_timer(0.5, self.publish_map)
        self.get_logger().info('Publishing 19x13 teaching map to /student/map')

    def publish_map(self):
        message = OccupancyGrid()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.info.resolution = RES
        message.info.width = W
        message.info.height = H
        message.info.origin.orientation.w = 1.0
        # ROS occupancy grids use x varying fastest in their 1D data array.
        message.data = [100 if (x,y) in WALLS else 0
                        for y in range(H) for x in range(W)]
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = MapTeacher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
