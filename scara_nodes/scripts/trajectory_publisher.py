#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/scara_trajectory_controller/joint_trajectory',
            1)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['joint1', 'joint2', 'joint3']
        point = JointTrajectoryPoint()
        point.positions = [random.random()*3-1.5, random.random()*3-1.5, 0.1+random.random()*0.2]
        point.time_from_start.sec = 1
        msg.points = [point]
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    trajectory_publisher = TrajectoryPublisher()

    rclpy.spin(trajectory_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
