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
        Node(
            package='areplusplus',
            executable='artag_detector_node',
            name='artag_detector_node',
            output='screen'
        )
    ])
