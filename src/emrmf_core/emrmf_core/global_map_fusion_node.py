#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import time

class GlobalMapFusionNode(Node):
    def __init__(self):
        super().__init__('global_map_fusion_node')

        # In a real scenario, this would subscribe to multiple incoming map topics from
        # the LoRa receiver node, e.g., /robot_1/map, /robot_2/map, etc.
        self.map_sub_1 = self.create_subscription(
            OccupancyGrid,
            '/robot_1/map',
            self.map_callback_robot1,
            10
        )

        self.map_sub_2 = self.create_subscription(
            OccupancyGrid,
            '/robot_2/map',
            self.map_callback_robot2,
            10
        )

        # Publisher for the fused global map
        self.global_map_pub = self.create_publisher(
            OccupancyGrid,
            '/emrmf/global_map',
            10
        )

        # Placeholders for storing incoming map data
        self.robot1_map = None
        self.robot2_map = None

        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info("Global Map Fusion Node Initialized.")

    def map_callback_robot1(self, msg):
        self.robot1_map = msg
        self.get_logger().debug("Received map from Robot 1.")

    def map_callback_robot2(self, msg):
        self.robot2_map = msg
        self.get_logger().debug("Received map from Robot 2.")

    def perform_g2o_pose_graph_optimization(self):
        """
        Placeholder for Phase 3: Pose Graph Optimization using g2o.
        The non-linear least squares optimization aims to minimize error
        over the combined global pose graph.
        """
        self.get_logger().info("Running g2o global pose graph optimization... (Simulated)")
        # Simulate computational delay for optimization
        time.sleep(1.0)
        return True

    def data_association_and_alignment(self):
        """
        Placeholder for Phase 3: Data Association and Alignment.
        Calculates the transformation matrix T that aligns sub-maps from different robots.
        """
        self.get_logger().info("Performing data association and nearest-neighbor feature matching... (Simulated)")
        return np.eye(4) # Return identity matrix as a placeholder

    def timer_callback(self):
        if self.robot1_map is not None and self.robot2_map is not None:
            # 1. Align submaps
            alignment_matrix = self.data_association_and_alignment()

            # 2. Run Pose Graph Optimization (g2o)
            optimization_success = self.perform_g2o_pose_graph_optimization()

            if optimization_success:
                # 3. Publish fused global map (Mock: just publishing robot 1's map for now)
                self.get_logger().info("Global Map Fusion Complete. Publishing /emrmf/global_map")
                self.global_map_pub.publish(self.robot1_map)

                # Reset maps to wait for next update
                self.robot1_map = None
                self.robot2_map = None

def main(args=None):
    rclpy.init(args=args)
    global_map_fusion_node = GlobalMapFusionNode()
    try:
        rclpy.spin(global_map_fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        global_map_fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
