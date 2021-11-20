import os
import rclpy
import pygame
from rclpy.node import Node
from geometry_msgs.msg import Twist

os.environ["SDL_VIDEODRIVER"] = "dummy"
pygame.init()
j = pygame.joystick.Joystick(0)
j.init()
print ('Initialized Joystick : %s' % j.get_name())

class TeleopGamepad(Node):
    
    def __init__(self):
        super().__init__('teleop_gamepad')
        self.pub_velocity = self.create_publisher(Twist, 'velocity', 10)
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        pygame.event.pump()
        
        
        velocity = Twist()
        velocity.linear.x = round(j.get_axis(1), 2) #Left thumbstick Y
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        
        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = round(j.get_axis(2), 2) #Right thumbstick X     
        
        print("Throttle:", velocity.linear.x)
        print("Steering:", velocity.linear.y) 
        
        self.pub_velocity.publish(velocity)
        
        
def main(args=None):
    rclpy.init(args=args)

    teleop_gamepad = TeleopGamepad()

    rclpy.spin(teleop_gamepad)
    
    teleop_gamepad.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()