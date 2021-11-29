import rclpy
import math

from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from adafruit_servokit import ServoKit

class MotorJetracer(Node):
    
    def __init__(self):
        super().__init__('motors')
        
        self.sub = self.create_subscription(Twist, 'velocity', self.twist_listener, 10)
        self.steering = traitlets.Float()
        self.throttle = traitlets.Float()
        self.i2c_address = traitlets.Integer(default_value=0x40)
        self.steering_gain = traitlets.Float(default_value=-0.65)
        self.steering_offset = traitlets.Float(default_value=0)
        self.steering_channel = traitlets.Integer(default_value=0)
        self.throttle_gain = traitlets.Float(default_value=0.8)
        self.throttle_channel = traitlets.Integer(default_value=1)
        self.kit = ServoKit(channels=16, address=self.i2c_address)
        self.steering_motor = self.kit.continuous_servo[self.steering_channel]
        self.throttle_motor = self.kit.continuous_servo[self.throttle_channel]

    def twist_listener(self, msg):

        self.throttle_motor.throttle = msg.linear.x * self.throttle_gain
        self.steering_motor.throttle = msg.angular.x * self.steering_gain + self.steering_offset
    
     
    def destroy_node(self):
        self.get_logger().info(f"shutting down, stopping robot...")
        self.stop()
    
    def stop(self):
        self.set_speed(0, 0)

def main(args=None):
    rclpy.init(args=args)

    node = MotorJetracer()
    node.get_logger().info("listening for velocity messages...")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("node stopped by keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()
     
if __name__ == '__main__':
    main()
    