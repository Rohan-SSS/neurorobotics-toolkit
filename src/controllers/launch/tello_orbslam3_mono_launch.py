import launch
import launch_ros

def generate_launch_description():
    logger_default = "INFO"
    logger = launch.substitutions.LaunchConfiguration("log-level", default = logger_default)
    logger_arg = launch.actions.DeclareLaunchArgument(
            "log-level",
            default_value=[logger_default],
            description="Logging level")
    nodes = [
        # Tello driver node
        launch_ros.actions.Node(
            package='tello',
            executable='tello',
            output='screen',
            namespace='/',
            name='tello',
            parameters=[
                {'connect_timeout': 10.0},
                {'tello_ip': '192.168.10.1'},
                {'tf_base': 'map'},
                {'tf_drone': 'drone'}
            ],
            remappings=[
                ('/image_raw', '/camera')
            ],
            respawn=True
        ),

        # Tello control node
        launch_ros.actions.Node(
            package='tello_control',
            executable='tello_control',
            namespace='/',
            name='control',
            output='screen',
            respawn=False
        ),
        
        # ORB SLAM 3 Controller
        # TODO set parameters for node through substitutions
        launch_ros.actions.Node(
            package='controllers',
            executable='orbslam3_monocular_controller',
            output='screen',
            namespace='/',
            name='orbslam3_controller',
            respawn=True,
            arguments=['--ros-args', '--log-level', logger],
            parameters = [{
                'settings_path': '/ws/ros_ws/src/slam/orb_slam3/config/Monocular/tello.yaml',
                'camera_topic': '/camera',
                'vocab_file_path': '/ws/ros_ws/src/slam/orb_slam3/Vocabulary/ORBvoc.txt.bin'
            }]
        ),

        # ORB SLAM3 compute node
        launch_ros.actions.Node(
            package='slam',
            executable='orbslam3_mono_node',
            name='orbslam3_mono_node',
            respawn=True,
            arguments=['--ros-args', '--log-level', logger]
        ),

        launch_ros.actions.Node(
            package='rqt_gui',
            executable='rqt_gui',
            output='screen',
            namespace='/',
            name='rqt',
            respawn=False
        ),

        # RViz data visualization tool
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            namespace='/',
            name='rviz2',
            respawn=True,
            arguments=['-d', '/ws/config/tello_rviz.rviz']
        ),

        # Static TF publisher
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace='/',
            name='tf',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'drone'],
            respawn=True
        ),
    ]


    return launch.LaunchDescription(nodes)
