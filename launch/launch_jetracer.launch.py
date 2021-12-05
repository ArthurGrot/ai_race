from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ai_race',
            namespace='ai_race',
            executable='teleop_gamepad',
            name='ai_race_teleop'
        ),
        Node(
            package='ai_race',
            namespace='ai_race',
            executable='motor_jetracer',
            name='ai_race_motor'
        )
        # ,
        # Node(
        #     package='ai_race',
        #     namespace='ai_race',
        #     executable='display',
        #     name='ai_race_display'
        # )
    ])