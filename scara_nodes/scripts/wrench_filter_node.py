#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from scipy import signal
from math import pi

from geometry_msgs.msg import WrenchStamped

class WrenchFilter(Node):

    def __init__(self):
        super().__init__('wrench_filter')
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/wrench',
            self.callback,
            1)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(WrenchStamped, '/wrench_filtered', 1)
        self.forceZ = 0.0
        self.cutoff_hz = 2
        self.dt = 0.001
        self.rc = 1 / (2*pi*self.cutoff_hz)
        self.alpha = 1 - self.dt / (self.dt + self.rc)

    def callback(self,msg):
        self.forceZ = self.alpha * self.forceZ + (1 - self.alpha) * msg.wrench.force.z
        msg.wrench.force.z = self.forceZ
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    wrench_filter = WrenchFilter()

    rclpy.spin(wrench_filter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wrench_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()