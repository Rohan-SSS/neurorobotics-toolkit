import launch
import launch_ros

def generate_launch_description():
    logger = launch.substitutions.LaunchConfiguration("log_level")
    return launch.LaunchDescription([
        # launch.actions.DeclareLaunchArgument(
        #     "log_level",
        #     default_value=["debug"],
        #     description="Logging level"),
        launch_ros.actions.Node(
            package='sensors',
            executable='realsense_talker',
            arguments=['--ros-args', '--log-level', "INFO"],
            output='screen',
            name='realsense_talker'),
        launch_ros.actions.Node(
            package='sensors',
            executable='lepton_talker',
            arguments=['--ros-args', '--log-level', "INFO"],
            output='screen',
            name='lepton_talker'),
        launch_ros.actions.Node(
            package='sensors',
            executable='kalibr_listener',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO'],
            name='kalibr_listener',
            parameters = [{
                "realsense_talker_node_name": "realsense_talker",
                "realsense_device_name": "Intel_RealSense_D455",
                "lepton_talker_node_name": "lepton_talker",
                "lepton_device_name": "device_0",
                "pattern_width": "7",
                "pattern_height": "6"
                }])
        ])
