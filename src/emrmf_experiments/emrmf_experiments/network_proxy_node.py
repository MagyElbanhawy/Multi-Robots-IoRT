import rclpy
from rclpy.node import Node
import random
import time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped

# Mapping of string types to ROS 2 message types
MESSAGE_TYPES = {
    'nav_msgs/msg/Odometry': Odometry,
    'geometry_msgs/msg/PoseStamped': PoseStamped,
    'sensor_msgs/msg/PointCloud2': PointCloud2,
    'std_msgs/msg/String': String
}

class TopicProxy:
    def __init__(self, node, input_topic, output_topic, msg_type_str, delay_sec, packet_loss_rate, noise_stddev):
        self.node = node
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.msg_type_str = msg_type_str
        self.delay_sec = delay_sec
        self.packet_loss_rate = packet_loss_rate
        self.noise_stddev = noise_stddev

        self.msg_type = MESSAGE_TYPES.get(self.msg_type_str)
        if not self.msg_type:
            self.node.get_logger().error(f"Unknown message type: {self.msg_type_str}")
            return

        self.publisher = self.node.create_publisher(self.msg_type, self.output_topic, 10)
        self.subscriber = self.node.create_subscription(
            self.msg_type,
            self.input_topic,
            self.callback,
            10
        )

        self.message_queue = []

    def callback(self, msg):
        # Apply packet loss
        if random.random() < self.packet_loss_rate:
            # Message dropped
            return

        # Apply noise if applicable
        if self.noise_stddev > 0.0:
            if isinstance(msg, Odometry):
                msg.pose.pose.position.x += random.gauss(0, self.noise_stddev)
                msg.pose.pose.position.y += random.gauss(0, self.noise_stddev)
                msg.pose.pose.position.z += random.gauss(0, self.noise_stddev)
            elif isinstance(msg, PoseStamped):
                msg.pose.position.x += random.gauss(0, self.noise_stddev)
                msg.pose.position.y += random.gauss(0, self.noise_stddev)
                msg.pose.position.z += random.gauss(0, self.noise_stddev)

        # Add to queue with target publish time
        publish_time = time.time() + self.delay_sec
        self.message_queue.append((publish_time, msg))

    def process_queue(self):
        current_time = time.time()
        # Find messages that are ready to be published
        messages_to_publish = []
        remaining_messages = []

        for target_time, msg in self.message_queue:
            if current_time >= target_time:
                messages_to_publish.append(msg)
            else:
                remaining_messages.append((target_time, msg))

        self.message_queue = remaining_messages

        for msg in messages_to_publish:
            self.publisher.publish(msg)

class NetworkProxyNode(Node):
    def __init__(self):
        super().__init__('network_proxy_node')

        # Declare parameters
        self.declare_parameter('delay_sec', 0.0)
        self.declare_parameter('packet_loss_rate', 0.0)
        self.declare_parameter('noise_stddev', 0.0)
        self.declare_parameter('topics_to_proxy', [])

        # Fallback dictionary of topics to proxy if param is empty or parsing fails
        # Format: "input_topic,output_topic,msg_type"
        self.declare_parameter('proxy_configs', [
            "/robot1/local_map,/proxy/robot1/local_map,sensor_msgs/msg/PointCloud2",
            "/robot2/local_map,/proxy/robot2/local_map,sensor_msgs/msg/PointCloud2",
            "/robot1/pose_update,/proxy/robot1/pose_update,nav_msgs/msg/Odometry",
            "/robot2/pose_update,/proxy/robot2/pose_update,nav_msgs/msg/Odometry",
            "/map_fusion/inter_robot_constraints,/proxy/map_fusion/inter_robot_constraints,geometry_msgs/msg/PoseStamped"
        ])

        self.delay_sec = self.get_parameter('delay_sec').value
        self.packet_loss_rate = self.get_parameter('packet_loss_rate').value
        self.noise_stddev = self.get_parameter('noise_stddev').value
        proxy_configs = self.get_parameter('proxy_configs').value

        self.get_logger().info(f"Initializing Network Proxy: Delay={self.delay_sec}s, Loss={self.packet_loss_rate*100}%, Noise={self.noise_stddev}m")

        self.proxies = []
        for config_str in proxy_configs:
            parts = config_str.split(',')
            if len(parts) == 3:
                input_topic = parts[0].strip()
                output_topic = parts[1].strip()
                msg_type = parts[2].strip()

                proxy = TopicProxy(self, input_topic, output_topic, msg_type, self.delay_sec, self.packet_loss_rate, self.noise_stddev)
                self.proxies.append(proxy)
                self.get_logger().info(f"Proxying: {input_topic} -> {output_topic} ({msg_type})")
            else:
                self.get_logger().warn(f"Invalid proxy config: {config_str}")

        # Timer to process message queues
        self.timer = self.create_timer(0.01, self.process_queues)

    def process_queues(self):
        for proxy in self.proxies:
            proxy.process_queue()

def main(args=None):
    rclpy.init(args=args)
    node = NetworkProxyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
