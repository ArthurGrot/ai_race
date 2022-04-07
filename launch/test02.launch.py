from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ai_race',
            namespace='ai_race_jetracer_1',
            executable='camera_pub',
            name='ai_race_camera_pub'
        ),
    ])