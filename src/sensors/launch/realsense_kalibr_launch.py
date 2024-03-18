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
            executable='kalibr_realsense_listener',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO'],
            name='kalibr_realsense_listener'),
  ])
