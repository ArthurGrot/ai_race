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
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        pygame.event.pump()
        
        throttle = j.get_axis(1) #Left thumbstick Y
        steering = j.get_axis(2) #Right thumbstick X
        
        self.pub_throttle.publish(throttle)
        self.pub_steering.publish(steering)
        
        print("Throttle:", throttle)
        print("Steering:", steering)   
        

def main(args=None):
    rclpy.init(args=args)

    teleop_gamepad = TeleopGamepad()

    rclpy.spin(teleop_gamepad)
    
    teleop_gamepad.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()