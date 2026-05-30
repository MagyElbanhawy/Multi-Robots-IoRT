from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    # Arguments
    p_arg = DeclareLaunchArgument('p', default_value='2.0')
    gamma_arg = DeclareLaunchArgument('gamma', default_value='0.1')
    tau_e_arg = DeclareLaunchArgument('tau_e', default_value='0.5')
    mode_arg = DeclareLaunchArgument('ablation_mode', default_value='full_emrmf')
    delay_arg = DeclareLaunchArgument('delay_sec', default_value='0.0')
    loss_arg = DeclareLaunchArgument('packet_loss_rate', default_value='0.0')
    run_id_arg = DeclareLaunchArgument('run_id', default_value='test_run')
    out_dir_arg = DeclareLaunchArgument('output_dir', default_value='/tmp/emrmf_experiments')

    # Network Proxy Node
    proxy_node = Node(
        package='emrmf_experiments',
        executable='network_proxy_node',
        name='network_proxy_node',
        output='screen',
        parameters=[{
            'delay_sec': LaunchConfiguration('delay_sec'),
            'packet_loss_rate': LaunchConfiguration('packet_loss_rate'),
            'proxy_configs': [
                "/robot1/local_map,/proxy/robot1/local_map,sensor_msgs/msg/PointCloud2",
                "/robot2/local_map,/proxy/robot2/local_map,sensor_msgs/msg/PointCloud2",
                "/robot1/pose_update,/proxy/robot1/pose_update,nav_msgs/msg/Odometry",
                "/robot2/pose_update,/proxy/robot2/pose_update,nav_msgs/msg/Odometry",
                "/map_fusion/inter_robot_constraints,/proxy/map_fusion/inter_robot_constraints,geometry_msgs/msg/PoseStamped"
            ]
        }]
    )

    # Logger Node
    logger_node = Node(
        package='emrmf_experiments',
        executable='experiment_logger_node',
        name='experiment_logger_node',
        output='screen',
        parameters=[{
            'p': LaunchConfiguration('p'),
            'gamma': LaunchConfiguration('gamma'),
            'tau_e': LaunchConfiguration('tau_e'),
            'ablation_mode': LaunchConfiguration('ablation_mode'),
            'delay_sec': LaunchConfiguration('delay_sec'),
            'packet_loss_rate': LaunchConfiguration('packet_loss_rate'),
            'run_id': LaunchConfiguration('run_id'),
            'output_dir': LaunchConfiguration('output_dir'),

            # Use proxied topics for logging if we want to monitor what the fusion node sees
            # However, typically we log the output of the SLAM system.
            'est_pose_topic': '/robot1/pose_update',
            'gt_pose_topic': '/ground_truth/pose',
            'pose_msg_type': 'nav_msgs/msg/Odometry',
            'source_points_topic': '/map_fusion/source_matched_points',
            'target_points_topic': '/map_fusion/target_matched_points'
        }]
    )

    # When the logger node exits (which happens when /experiment_done is received),
    # emit a Shutdown event to gracefully shut down the entire launch file.
    shutdown_on_logger_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=logger_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    return LaunchDescription([
        p_arg, gamma_arg, tau_e_arg, mode_arg, delay_arg, loss_arg, run_id_arg, out_dir_arg,
        proxy_node,
        logger_node,
        shutdown_on_logger_exit
    ])
