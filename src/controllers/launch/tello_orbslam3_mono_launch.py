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
            package='tello_driver',
            executable='tello_driver_main',
            output='screen',
            namespace='/',
            arguments=['--ros-args', '--log-level', logger],
            name='tello',
            remappings=[
                ('/image_raw', '/camera')
            ],
            respawn=True
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
    ]


    return launch.LaunchDescription(nodes)
