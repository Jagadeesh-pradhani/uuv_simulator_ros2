#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from copy import deepcopy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32
from tf_quaternion.transformations import euler_from_quaternion

class PublishFootprints(Node):
    def __init__(self):
        super().__init__('publish_footprints')
        self.vehicle_pub = dict()   # Dictionary for publishers keyed by model name.
        self.odom_sub = dict()      # Dictionary for subscribers.
        self.marker = np.array([[0, 0.75], [-0.5, -0.25], [0.5, -0.25]])
        # Timer to update vehicle list (dynamic model discovery not implemented)
        self.create_timer(10.0, self.update_vehicle_list)

    def rot(self, alpha):
        return np.array([[np.cos(alpha), -np.sin(alpha)],
                         [np.sin(alpha),  np.cos(alpha)]])
    
    def odometry_callback(self, msg, name):
        if name not in self.vehicle_pub:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = [msg.pose.pose.orientation.x,
                       msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z,
                       msg.pose.pose.orientation.w]
        yaw = euler_from_quaternion(orientation)[2]
        new_marker = deepcopy(self.marker)
        points = []
        for i in range(new_marker.shape[0]):
            new_marker[i, :] = np.dot(self.rot(yaw - np.pi/2), new_marker[i, :])
            new_marker[i, 0] += x
            new_marker[i, 1] += y
            p = Point32()
            p.x = new_marker[i, 0]
            p.y = new_marker[i, 1]
            points.append(p)
        new_poly = PolygonStamped()
        new_poly.header.stamp = self.get_clock().now().to_msg()
        new_poly.header.frame_id = 'world'
        new_poly.polygon.points = points
        self.vehicle_pub[name].publish(new_poly)

    def update_vehicle_list(self):
        # In ROS2, dynamic topic discovery must use rclpy APIs.
        # For now, we simply log that this function was called.
        self.get_logger().info("update_vehicle_list called. Dynamic model discovery is not implemented.")

def main(args=None):
    rclpy.init(args=args)
    node = PublishFootprints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
