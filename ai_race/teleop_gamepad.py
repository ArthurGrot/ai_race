import os
import rclpy
import pygame
from rclpy.node import Node
from std_msgs.msg import Float32

os.environ["SDL_VIDEODRIVER"] = "dummy"
pygame.init()
j = pygame.joystick.Joystick(0)
j.init()
print ('Initialized Joystick : %s' % j.get_name())

class TeleopGamepad(Node):
    
    def __init__(self):
        super().__init__('teleop_gamepad')
        self.pub_throttle = self.create_publisher(Float32, 'throttle', 10)
        self.pub_steering = self.create_publisher(Float32, 'steering', 10)
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        pygame.event.pump()
        
        throttle = Float32()
        throttle.data = round(j.get_axis(1), 2) #Left thumbstick Y
        steering = Float32()
        steering.data = round(j.get_axis(2), 2) #Right thumbstick X        
        
        print("Throttle:", throttle.data)
        print("Steering:", steering.data) 
        
        self.pub_throttle.publish(throttle)
        self.pub_steering.publish(steering)
        
        
def main(args=None):
    rclpy.init(args=args)

    teleop_gamepad = TeleopGamepad()

    rclpy.spin(teleop_gamepad)
    
    teleop_gamepad.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()