import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_emrmf_experiments = get_package_share_directory('emrmf_experiments')

    robot_count_arg = DeclareLaunchArgument('robot_count', default_value='2')
    lidar_noise_arg = DeclareLaunchArgument('lidar_noise', default_value='0.01')

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={'world': os.path.join(pkg_emrmf_experiments, 'worlds', 'emrmf_indoor.world')}.items()
    )

    def spawn_robots(context, *args, **kwargs):
        count = int(LaunchConfiguration('robot_count').perform(context))
        lidar_noise = LaunchConfiguration('lidar_noise').perform(context)

        nodes = []
        urdf_path = os.path.join(pkg_emrmf_experiments, 'urdf', 'emrmf_robot.urdf.xacro')

        for i in range(1, count + 1):
            robot_name = f'robot{i}'
            x_pos = (i - 1) * 2.0 - ((count-1)*1.0) # Spread them out
            y_pos = 0.0

            # Robot state publisher
            rsp_node = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=robot_name,
                parameters=[{'robot_description': Command(['xacro ', urdf_path, f' robot_namespace:={robot_name}', f' lidar_noise:={lidar_noise}']) }]
            )

            # Gazebo spawner
            spawn_node = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', robot_name,
                           '-topic', f'/{robot_name}/robot_description',
                           '-x', str(x_pos), '-y', str(y_pos), '-z', '0.1'],
                output='screen'
            )

            nodes.extend([rsp_node, spawn_node])

        return nodes

    return LaunchDescription([
        robot_count_arg,
        lidar_noise_arg,
        gazebo,
        OpaqueFunction(function=spawn_robots)
    ])
