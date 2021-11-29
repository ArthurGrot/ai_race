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
        
        self.declare_parameter('steering', 0.0)
        self.declare_parameter('throttle', 0.0)
        self.declare_parameter('steering_gain', 0.8)
        self.declare_parameter('steering_offset', 0.15)
        self.declare_parameter('throttle_gain', 0.3)
        
        self.steering = self.get_parameter('steering').value
        self.throttle = self.get_parameter('throttle').value
        self.steering_gain = self.get_parameter('steering_gain').value
        self.steering_offset = self.get_parameter('steering_offset').value
        self.throttle_gain = self.get_parameter('throttle_gain').value

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.last_x = -999
        self.last_rot = -999

        self.i2c_address = 0x40
        self.steering_channel = 0
        self.throttle_channel = 1

        self.kit = ServoKit(channels=16, address=self.i2c_address)
        self.steering_motor = self.kit.continuous_servo[self.steering_channel]
        self.throttle_motor = self.kit.continuous_servo[self.throttle_channel]

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'steering':
                self.steering = param.value
            elif param.name == 'throttle':
                self.throttle = param.value
            elif param.name == 'steering_gain':
                self.steering_gain = param.value
            elif param.name == 'steering_offset':
                self.steering_offset = param.value
            elif param.name == 'throttle_gain':
                self.throttle_gain = param.value
            else:
                raise ValueError(f'unknown parameter {param.name}')
                
        return SetParametersResult(successful=True)

    def twist_listener(self, msg):
        x = msg.linear.x * self.throttle_gain * -1.0
        rot = msg.angular.z * self.steering_gain * -1.0 + self.steering_offset

        if x == self.last_x and rot == self.last_rot:
            return

        self.last_x = x
        self.last_rot = rot

        self.get_logger().info(f"x={x:.03f} rotation={rot:.03f}")

        if x < -1.0:
            x = -1.0
        elif x > 1.0:
            x = 1.0
        
        if rot < -1.0:
            rot = -1.0
        elif rot > 1.0:
            rot = 1.0

        self.throttle_motor.throttle = x
        self.steering_motor.throttle =  rot 

        self.get_logger().info(f"x={x:.03f} rotation={rot:.03f}")
     
    def destroy_node(self):
        self.get_logger().info(f"shutting down, stopping robot...")
        self.stop()
    
    def stop(self):
        self.throttle_motor.throttle = 0.0
        self.steering_motor.throttle = 0.0

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
    