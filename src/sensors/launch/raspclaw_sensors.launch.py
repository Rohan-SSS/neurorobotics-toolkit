import os
import launch
import launch_ros
import ament_index_python

def generate_launch_description() -> launch.launch_description.LaunchDescription:
    # TODO add diagnositic nodes in the launch container, all of which will be run alongside the sensor
    # TODO find and add camera params file to be used by camera_ros. Need to figure out where the file should be placed
    camera_container = launch_ros.actions.ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package='camera_ros',
                plugin='camera::CameraNode',
                parameters=[{
                    "camera": 0,
                    "width": 640,
                    "height": 480,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )

    # TODO create composable node with diagnositic nodes for IMU sensor
    mpu6050_dir = ament_index_python.packages.get_package_share_directory('ros2_mpu6050')
    mpu6050_included_launch_description = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(mpu6050_dir, 'launch/ros2_mpu6050.launch.py')
        )
    )

    return launch.launch_description.LaunchDescription([
        camera_container,
        mpu6050_included_launch_description
    ])

