import launch
import launch_ros

def generate_launch_description():
    # Declare a launch argument for log level
    log_level_arg = launch.actions.DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for the node'
    )

    # Launch configuration for the log level
    log_level = launch.substitutions.LaunchConfiguration('log_level')

    # Define the OctoMap node
    octomap_node = launch_ros.actions.Node(
        package='frontier_detection',  # Replace with your package name
        executable='octomap_node',   # Replace with your node executable
        name='octomap_node',
        output='screen',
        parameters=[],  # Add any parameters if required
        arguments=['--ros-args', '--log-level', log_level]
    )

    return launch.LaunchDescription([
        log_level_arg,  # Include the log level argument
        octomap_node    # Launch the OctoMap node
    ])

