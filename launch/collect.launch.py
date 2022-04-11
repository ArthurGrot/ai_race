from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ai_race',
            namespace='ai_race_jetracer_1',
            executable='teleop_gamepad',
            name='ai_race_teleop'
        ),
        Node(
            package='ai_race',
            namespace='ai_race_jetracer_1',
            executable='motor_jetracer',
            name='ai_race_motor'
        ),
        Node(
            package='ai_race',
            namespace='ai_race_jetracer_1',
            executable='motor_processing',
            name='ai_race_motor_processing'
        ),
    ])