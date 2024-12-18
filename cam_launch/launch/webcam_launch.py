from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_tools',
            executable='cam2image',
            name='cam2image',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'frequency': 30.0},  # Frequency of image capture
                {'width': 640},       # Width of the captured image
                {'height': 480},       # Height of the captured image
                {'video_device': '/dev/video0'}  # Set to your external webcam device path
            ],
        ),
        Node(
            package='image_view',
            executable='image_view',
            name='image_view',
            output='screen',
            remappings=[
                ('image', '/image')  # Ensure this matches the topic from cam2image
            ],
        ),
    ])

