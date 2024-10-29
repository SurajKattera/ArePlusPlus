from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='areplusplus',
            executable='TaskHandler',
            name='task_handler',
            output='screen'
        ),
        # Node(
        #     package='areplusplus',
        #     executable='artag_detector_node',
        #     name='artag_detector_node',
        #     output='screen'
        # ), 
        # Launch AprilTag Node from apriltag_ros package
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            remappings=[
                ('image_rect', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info')
            ],
            parameters=[
                {'family': '36h11'},
                {'size': 0.5},
                {'max_hamming': 0}
            ]
        )
    ])
