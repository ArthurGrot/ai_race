import os
import rclpy
import pygame
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

os.environ["SDL_VIDEODRIVER"] = "dummy"
pygame.init()
j = pygame.joystick.Joystick(0)
j.init()
print ('Initialized Joystick : %s' % j.get_name())


class TeleopGamepad(Node):
    
    def __init__(self):
        super().__init__('teleop_gamepad')
        self.pub_velocity = self.create_publisher(Twist, 'velocity', 10)
        self.pub_line_following_mode = self.create_publisher(Bool, 'line_following_mode', 10)
        self.line_following_mode = Bool()
        self.line_following_mode.data = False
        
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        pygame.event.pump()
        
        if self.line_following_mode.data == False:
            velocity = Twist()
            velocity.linear.x = round(j.get_axis(1), 2) / 2 #Left thumbstick Y
            velocity.linear.y = 0.0
            velocity.linear.z = 0.0
            
            velocity.angular.x = 0.0
            velocity.angular.y = 0.0
            velocity.angular.z = round(j.get_axis(2), 2) #Right thumbstick X     
            
            #self.get_logger().info(f"Throttle: {velocity.linear.x}")
            #self.get_logger().info(f"Steering: {velocity.linear.y}") 
            
            self.pub_velocity.publish(velocity)


        for event in pygame.event.get(): 
            if event.type == pygame.JOYBUTTONDOWN:
                # buttons = j.get_numbuttons()
                # self.get_logger().info("Number of buttons: {}".format(buttons))

                # for i in range(buttons):
                #     button = j.get_button(i)
                #     self.get_logger().info("Button {:>2} value: {}".format(i, button))

                button = j.get_button(11)

                if button == True:
                    self.line_following_mode.data = not self.line_following_mode.data
                    self.pub_line_following_mode.publish(self.line_following_mode)
                    self.get_logger().info(str(self.line_following_mode.data))
            
        
def main(args=None):
    rclpy.init(args=args)

    teleop_gamepad = TeleopGamepad()

    try:
        rclpy.spin(teleop_gamepad)
    except KeyboardInterrupt:
        teleop_gamepad.get_logger().info("node stopped by keyboard interrupt")
    finally:        
        teleop_gamepad.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()