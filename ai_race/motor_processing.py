import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class MotorProcessing(Node):
    def __init__(self):
        super().__init__('motor_processing')
        # Data from Twists
            # yolo speed
        self.yolo_speed = -0.4 # speed 30 
        self.yolo_angle = 0.0 
            # remote
        self.remote_mode = Bool()
        self.remote_mode.data = True

        self.remote_speed = 0.0
        self.remote_angle = 0.0
            # line
        self.line_angle = 0.0

        self.yolo_avoidance_mode = False
        
        # subscriptions 

            # yolo 
        self.sub_yolo_speed = self.create_subscription(Twist, 'speed_yolo', self.steering_yolo_speed, 1)
            # line
        self.sub_line = self.create_subscription(Twist, 'steering_line', self.steering_line, 1)
        
            # remote
        self.sub_mode = self.create_subscription(Bool, 'remote_mode', self.remote_mode_callback, 10)
        self.sub_remote = self.create_subscription(Twist, 'steering_remote', self.steering_remote, 1)
        
        
        # publisher
        self.pub = self.create_publisher(Twist, 'velocity', 1)


    def remote_mode_callback(self, msg):
        self.remote_mode.data = msg.data
        self.publisher()

    # publisher motor
    def publisher(self):
        publish_value = self.decision_maker()
        self.pub.publish(publish_value)

    def decision_maker(self):
        publish_value = Twist()
        # remote control
        if self.remote_mode.data == True:
            publish_value.linear.x = self.remote_speed
            publish_value.angular.z = self.remote_angle
            return publish_value

        # line following / yolo
        else:
            publish_value.linear.x = self.yolo_speed
            # line following
            if self.yolo_avoidance_mode:+
                publish_value.angular.z = self.yolo_angle
            else 
                publish_value.angular.z = self.line_angle

            return publish_value



    # steering subscriptions
        # line follower
    def steering_line(self,msg):
        self.line_angle = msg.angular.z

        if self.remote_mode.data == False:
            self.publisher()
            
        # remote
    def steering_remote(self, msg):
        self.remote_speed = msg.linear.x
        self.remote_angle = msg.angular.z
        self.publisher()
        
        # yolo speed
    def steering_yolo_speed(self, msg):
        # speed
        self.yolo_speed = msg.linear.x
        # angle
        self.yolo_angle = msg.angular.z
        # yolo take the wheel https://youtu.be/lydBPm2KRaU?t=86
        self.yolo_avoidance_mode = bool(msg.linear.z)

        self.publisher()

def main(args=None):
    rclpy.init(args=args)

    node = MotorProcessing()
    node.get_logger().info("listening for all messages...")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("node stopped by keyboard interrupt")
    finally:
        rclpy.shutdown()
     
if __name__ == '__main__':
    main()
    