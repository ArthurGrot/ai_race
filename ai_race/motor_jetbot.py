import rclpy
import math

from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from Adafruit_MotorHAT import Adafruit_MotorHAT

MOTOR_LEFT = 1      # left motor ID
MOTOR_RIGHT = 2     # right motor ID

class MotorJetbot(Node):
    
    def __init__(self):
        super().__init__('motors')
        
        self.sub = self.create_subscription(Twist, 'velocity', self.twist_listener, 10)
        
        self.declare_parameter('left_trim', 0.0)
        self.declare_parameter('right_trim', 0.0)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('max_rpm', 200)              # https://www.adafruit.com/product/3777
        self.declare_parameter('wheel_separation', 0.1016)  # 10,16   cm
        self.declare_parameter('wheel_diameter', 0.060325)  #  6,0325 cm
        self.declare_parameter('rotation_speed_multiplier', 1)
        self.declare_parameter('speed_muliplier', 0.2)
        
        self.left_trim = self.get_parameter('left_trim').value
        self.right_trim = self.get_parameter('right_trim').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.rotation_speed_multiplier = self.get_parameter('rotation_speed_multiplier').value
        self.speed_muliplier = self.get_parameter('speed_muliplier').value
        
        self.add_on_set_parameters_callback(self.parameters_callback)
         
        self.last_x = -999
        self.last_rot = -999
        
        # open Adafruit MotorHAT driver
        self.driver = Adafruit_MotorHAT(i2c_bus=1)
        
        # get motor objects from driver
        self.motors = {
            MOTOR_LEFT : self.driver.getMotor(MOTOR_LEFT),
            MOTOR_RIGHT : self.driver.getMotor(MOTOR_RIGHT)
        }
        
        self.pwm_channels = {
            MOTOR_LEFT : (1, 0),
            MOTOR_RIGHT : (2, 3)
        }
        
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'left_trim':
                self.left_trim = param.value
            elif param.name == 'right_trim':
                self.right_trim = param.value
            elif param.name == 'max_pwm':
                self.max_pwm = param.value
            elif param.name == 'wheel_separation':
                self.wheel_separation = param.value
            elif param.name == 'rotation_speed_multiplier':
                self.rotation_speed_multiplier = param.value
            elif param.name == 'speed_muliplier':
                self.speed_muliplier = param.value
            else:
                raise ValueError(f'unknown parameter {param.name}')
                
        return SetParametersResult(successful=True)
        
    def twist_listener(self, msg):
        x = msg.linear.x * self.speed_muliplier
        rot = msg.angular.z
        
        if x == self.last_x and rot == self.last_rot:
            return
            
        self.last_x = x
        self.last_rot = rot
        
        left = x - rot * self.wheel_separation * self.rotation_speed_multiplier
        right = x + rot * self.wheel_separation * self.rotation_speed_multiplier
        
        # convert velocities to [-1,1]
        max_speed = (self.max_rpm / 60.0) * 2.0 * math.pi * (self.wheel_diameter * 0.5)

        """
        Sets the motor speeds between [-1.0, 1.0]
        """
        left = max(min(left, max_speed), -max_speed) / max_speed
        right = max(min(right, max_speed), -max_speed) / max_speed
        
        self.get_logger().info(f"x={x:.03f} rotation={rot:.03f} -> left={left:.03f} right={right:.03f}  (max_speed={max_speed:.03f} m/s)")
        
        self.set_speed(left, right)
        
    def set_speed(self, left, right):
        """
        Sets the motor speeds between [-1.0, 1.0]
        """
        self._set_pwm(MOTOR_LEFT, left, self.left_trim)
        self._set_pwm(MOTOR_RIGHT, right, self.right_trim)
        
    def _set_pwm(self, motor, value, trim):
        # apply trim and convert [-1,1] to PWM value
        pwm = int(min(max((abs(value) + trim) * self.max_pwm, 0), self.max_pwm))
        self.motors[motor].setSpeed(pwm)

        # set the motor direction
        ina, inb = self.pwm_channels[motor]
        
        if value > 0:
            self.motors[motor].run(Adafruit_MotorHAT.FORWARD)
            self.driver._pwm.setPWM(ina, 0, pwm * 16)
            self.driver._pwm.setPWM(inb, 0, 0)
        elif value < 0:
            self.motors[motor].run(Adafruit_MotorHAT.BACKWARD)
            self.driver._pwm.setPWM(ina, 0, 0)
            self.driver._pwm.setPWM(inb, 0, pwm * 16)
        else:
            self.motors[motor].run(Adafruit_MotorHAT.RELEASE)
            self.driver._pwm.setPWM(ina, 0, 0)
            self.driver._pwm.setPWM(inb, 0, 0)
     
    def destroy_node(self):
        self.get_logger().info(f"shutting down, stopping robot...")
        self.stop()
    
    def stop(self):
        self.set_speed(0, 0)

def main(args=None):
    rclpy.init(args=args)

    node = MotorJetbot()
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
    