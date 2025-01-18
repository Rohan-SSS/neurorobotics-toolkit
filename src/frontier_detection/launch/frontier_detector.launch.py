from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('frontier_detection'),
        'config',
        'config.rviz'
    ])

    return LaunchDescription([
        Node(
            package='frontier_detection',
            executable='global_frontier_detector',
            name='global_frontier_detector',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='frontier_detection',
            executable='local_frontier_detector',
            name='local_frontier_detector',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])
