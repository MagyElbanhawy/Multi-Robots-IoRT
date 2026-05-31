import rclpy
from rclpy.node import Node
import math
import numpy as np
import time
import csv
import os
import sys

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class ExperimentLoggerNode(Node):
    def __init__(self):
        super().__init__('experiment_logger_node')

        # Experiment parameters
        self.declare_parameter('p', 2.0)
        self.declare_parameter('gamma', 0.1)
        self.declare_parameter('tau_e', 0.5)
        self.declare_parameter('ablation_mode', 'full_emrmf')
        self.declare_parameter('delay_sec', 0.0)
        self.declare_parameter('packet_loss_rate', 0.0)
        self.declare_parameter('noise_stddev', 0.0)
        self.declare_parameter('output_dir', '/tmp/emrmf_experiments')
        self.declare_parameter('run_id', 'run_0')
        self.declare_parameter('robot_count', 1)

        # Topic parameters
        self.declare_parameter('pose_msg_type', 'nav_msgs/msg/Odometry')
        self.declare_parameter('source_points_topic', '/map_fusion/source_matched_points')
        self.declare_parameter('target_points_topic', '/map_fusion/target_matched_points')

        self.p = self.get_parameter('p').value
        self.gamma = self.get_parameter('gamma').value
        self.tau_e = self.get_parameter('tau_e').value
        self.ablation_mode = self.get_parameter('ablation_mode').value
        self.delay_sec = self.get_parameter('delay_sec').value
        self.packet_loss_rate = self.get_parameter('packet_loss_rate').value
        self.noise_stddev = self.get_parameter('noise_stddev').value
        self.output_dir = self.get_parameter('output_dir').value
        self.run_id = self.get_parameter('run_id').value
        self.robot_count = self.get_parameter('robot_count').value

        os.makedirs(self.output_dir, exist_ok=True)

        # State tracking
        self.est_poses = {i: [] for i in range(1, self.robot_count + 1)}
        self.gt_poses = {i: [] for i in range(1, self.robot_count + 1)}

        self.thetas = []
        self.source_clouds = []
        self.target_clouds = []
        self.fusion_times = []
        self.accepted_constraints = 0

        self.last_predicted_transforms = {i: None for i in range(1, self.robot_count + 1)}

        # Subscriptions
        pose_msg_type_str = self.get_parameter('pose_msg_type').value
        pose_type = Odometry if pose_msg_type_str == 'nav_msgs/msg/Odometry' else PoseStamped

        self.est_subs = []
        self.gt_subs = []

        for i in range(1, self.robot_count + 1):
            est_topic = f'/robot{i}/pose_update'
            gt_topic = f'/robot{i}/ground_truth'

            # Using default arguments in lambdas to capture the current robot index
            self.est_subs.append(self.create_subscription(
                pose_type, est_topic, lambda msg, r_id=i: self.est_pose_cb(msg, r_id), 10))
            self.gt_subs.append(self.create_subscription(
                pose_type, gt_topic, lambda msg, r_id=i: self.gt_pose_cb(msg, r_id), 10))

        self.src_pts_sub = self.create_subscription(
            PointCloud2, self.get_parameter('source_points_topic').value, self.src_pts_cb, 10)
        self.tgt_pts_sub = self.create_subscription(
            PointCloud2, self.get_parameter('target_points_topic').value, self.tgt_pts_cb, 10)

        # Stop signal
        self.done_sub = self.create_subscription(
            Bool, '/experiment_done', self.done_cb, 10)

        self.get_logger().info(f"Logger Initialized: Mode={self.ablation_mode}, p={self.p}, gamma={self.gamma}")
        self.experiment_active = True

    def est_pose_cb(self, msg, robot_id):
        if not self.experiment_active: return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = msg.pose.pose.position if isinstance(msg, Odometry) else msg.pose.position
        self.est_poses[robot_id].append((t, pos.x, pos.y, pos.z))

        if self.last_predicted_transforms[robot_id] is None:
            self.last_predicted_transforms[robot_id] = (pos.x, pos.y, pos.z)
        else:
            if self.ablation_mode in ['full_emrmf', 'trust_only']:
                observed_x, observed_y, observed_z = pos.x, pos.y, pos.z
                pred_x, pred_y, pred_z = self.last_predicted_transforms[robot_id]

                e_ij = math.sqrt((observed_x - pred_x)**2 + (observed_y - pred_y)**2 + (observed_z - pred_z)**2)
                current_time = self.get_clock().now().nanoseconds / 1e9
                delta_t = current_time - t
                if delta_t < 0: delta_t = 0.0

                term1 = max(0.0, 1.0 - (e_ij / self.tau_e)**self.p)
                term2 = math.exp(-self.gamma * delta_t)
                theta = term1 * term2
                self.thetas.append(theta)

                if theta > 0.3: # Arbitrary threshold for accepting constraint
                    self.accepted_constraints += 1

                self.last_predicted_transforms[robot_id] = (pos.x, pos.y, pos.z)

    def gt_pose_cb(self, msg, robot_id):
        if not self.experiment_active: return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = msg.pose.pose.position if isinstance(msg, Odometry) else msg.pose.position
        self.gt_poses[robot_id].append((t, pos.x, pos.y, pos.z))

    def src_pts_cb(self, msg):
        if not self.experiment_active: return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self.source_clouds.append((t, pts))
        self.fusion_times.append(time.time()) # Track rate of fusion

    def tgt_pts_cb(self, msg):
        if not self.experiment_active: return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self.target_clouds.append((t, pts))

    def done_cb(self, msg):
        if msg.data and self.experiment_active:
            self.get_logger().info("Experiment Done signal received. Saving results...")
            self.save_results()
            self.experiment_active = False

            # Shut down gracefully so the orchestrator can continue immediately
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)

    def compute_pose_rmse(self):
        all_errors = []
        for r_id in range(1, self.robot_count + 1):
            est = self.est_poses[r_id]
            gt = self.gt_poses[r_id]
            if not est or not gt: continue

            for t, x, y, z in est:
                closest_gt = min(gt, key=lambda g: abs(g[0] - t))
                if abs(closest_gt[0] - t) < 1.0:
                    sq_err = (x - closest_gt[1])**2 + (y - closest_gt[2])**2 + (z - closest_gt[3])**2
                    all_errors.append(sq_err)

        if not all_errors: return 0.0
        return math.sqrt(sum(all_errors) / len(all_errors))

    def compute_map_alignment_rmse(self):
        if not self.source_clouds or not self.target_clouds:
            return 0.0

        # Simplified: match latest clouds
        _, src_pts = self.source_clouds[-1]
        _, tgt_pts = self.target_clouds[-1]

        if len(src_pts) == 0 or len(tgt_pts) == 0:
            return 0.0

        # Compute nearest neighbor RMSE
        errors = []
        for sx, sy, sz in src_pts:
            min_dist_sq = min((sx-tx)**2 + (sy-ty)**2 + (sz-tz)**2 for tx, ty, tz in tgt_pts)
            errors.append(min_dist_sq)

        return math.sqrt(sum(errors) / len(errors))

    def save_results(self):
        pose_rmse = self.compute_pose_rmse()
        map_rmse = self.compute_map_alignment_rmse()

        theta_mean = float(np.mean(self.thetas)) if self.thetas else 0.0
        theta_std = float(np.std(self.thetas)) if self.thetas else 0.0

        avg_fusion_time = 0.0
        if len(self.fusion_times) > 1:
            diffs = [self.fusion_times[i] - self.fusion_times[i-1] for i in range(1, len(self.fusion_times))]
            avg_fusion_time = float(np.mean(diffs))

        csv_file = os.path.join(self.output_dir, 'raw_results.csv')
        write_header = not os.path.exists(csv_file)

        success_rate = 1.0 if map_rmse < 0.2 and pose_rmse < 0.5 else 0.0

        with open(csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow(['run_id', 'robot_count', 'ablation_mode', 'p', 'gamma', 'delay', 'packet_loss', 'noise',
                               'pose_rmse', 'map_alignment_rmse', 'fusion_time', 'theta_mean', 'theta_std', 'accepted_constraints', 'success_rate'])
            writer.writerow([
                self.run_id, self.robot_count, self.ablation_mode, self.p, self.gamma, self.delay_sec, self.packet_loss_rate, self.noise_stddev,
                pose_rmse, map_rmse, avg_fusion_time, theta_mean, theta_std, self.accepted_constraints, success_rate
            ])

        self.get_logger().info(f"Results saved to {csv_file}")

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.experiment_active:
            node.save_results()
    except SystemExit:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
