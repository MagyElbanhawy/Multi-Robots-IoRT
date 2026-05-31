#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import argparse

class TrajectoryDriver(Node):
    def __init__(self, robot_count):
        super().__init__('trajectory_driver')
        self.robot_count = robot_count
        self.pubs = []
        for i in range(1, self.robot_count + 1):
            pub = self.create_publisher(Twist, f'/robot{i}/cmd_vel', 10)
            self.pubs.append(pub)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def timer_callback(self):
        t = (self.get_clock().now().nanoseconds / 1e9) - self.start_time

        for i, pub in enumerate(self.pubs):
            msg = Twist()
            # Generate different overlapping trajectories based on robot index
            # Simple circular/figure-8 motions
            if i % 2 == 0:
                msg.linear.x = 0.5
                msg.angular.z = 0.5 * math.sin(t * 0.5) # Figure 8
            else:
                msg.linear.x = 0.4
                msg.angular.z = 0.6 * math.cos(t * 0.4) # Slightly different pattern

            pub.publish(msg)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot_count', type=int, default=2)
    # This might conflict with ros2 run passing remaining args.
    # Use standard rclpy parameter if integrated into launch, but keeping simple script execution for now.
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = TrajectoryDriver(args.robot_count)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
