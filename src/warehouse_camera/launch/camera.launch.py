from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            # This name 'camera_node' creates the /camera_node/... topics
            name='camera_node',
            output='screen',
            parameters=[
                # Corrected parameter names
                {'camera': 0},
                {'format': 'YUYV'}, # Or BGR888, whichever you prefer
                
                # These are correct
                {'image_width': 1920},
                {'image_height': 1080},
                {'framerate': 30.0},
            ]
        )
    ])