#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class TrustFactorNode(Node):
    def __init__(self):
        super().__init__('trust_factor_node')

        # Subscriber to local odometry (e.g., from robot_localization ekf)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        # Publisher for the trusted/weighted pose
        self.trusted_pose_pub = self.create_publisher(
            Odometry,
            '/emrmf/trusted_pose',
            10
        )

        self.prev_pose = None
        self.get_logger().info("Trust Factor Node Initialized. Subscribing to /odometry/filtered")

    def calculate_trust_factor(self, current_pose, prev_pose):
        """
        Placeholder for the Phase 2 Trust Factor (θ) calculation from the EMRMF paper.
        θ = [ (2 * |x_t - x_{t-1}| * |x~_2 - x~_1|) / (|x_t - x_{t-1}|^2 + |x~_2 - x~_1|^2) ]^3

        This mock implementation calculates a simple distance-based trust factor
        to simulate penalizing high-noise/high-latency jumps.
        """
        if prev_pose is None:
            return 1.0 # High trust on initial pose

        dx = current_pose.position.x - prev_pose.position.x
        dy = current_pose.position.y - prev_pose.position.y
        dz = current_pose.position.z - prev_pose.position.z

        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        # Mock logic: if distance jump is suspiciously high (> 0.5m), lower the trust factor
        if distance > 0.5:
            theta = 0.5  # Penalize trust
        else:
            theta = 1.0  # Full trust

        # In a real scenario, theta would be used to inflate the covariance matrix
        # before sending it to the global pose graph optimizer.
        return theta

    def odom_callback(self, msg):
        current_pose = msg.pose.pose

        theta = self.calculate_trust_factor(current_pose, self.prev_pose)

        # Create a new message to publish with inflated covariance based on theta
        trusted_msg = msg

        # Inflate covariance if trust factor is low (mock implementation)
        # Assuming msg.pose.covariance is a 36-element tuple/list
        covariance = list(trusted_msg.pose.covariance)
        if len(covariance) == 36:
            # Scale positional covariances (indices 0, 7, 14) inversely proportional to theta
            scale_factor = 1.0 / max(theta, 0.01)
            covariance[0] *= scale_factor
            covariance[7] *= scale_factor
            covariance[14] *= scale_factor
            trusted_msg.pose.covariance = tuple(covariance)

        self.trusted_pose_pub.publish(trusted_msg)

        self.get_logger().debug(f"Calculated Trust Factor: {theta:.2f}, Publishing to /emrmf/trusted_pose")
        self.prev_pose = current_pose

def main(args=None):
    rclpy.init(args=args)
    node = TrustFactorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
