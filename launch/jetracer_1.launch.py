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
            executable='camera_pub',
            name='ai_race_camera_pub'
        ),
        Node(
            package='ai_race',
            namespace='ai_race_jetracer_1',
            executable='flask_server',
            name='ai_race_flask_server'
        ),
        Node(
            package='ai_race',
            namespace='ai_race_jetracer_1',
            executable='line_follower',
            name='ai_race_line_follower'
        ),
        Node(
            package='ai_race',
            namespace='ai_race_jetracer_1',
            executable='motor_processing',
            name='ai_race_motor_processing'
        ),
        Node(
            package='ai_race',
            namespace='ai_race_jetracer_1',
            executable='yolo_jetracer',
            name='ai_race_yolo_jetracer'
        ),
        # ,
        # Node(
        #     package='ai_race',
        #     namespace='ai_race',
        #     executable='display',
        #     name='ai_race_display'
        # )
    ])