from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy
import cv_bridge
import rclpy
import cv2
from rclpy.node import Node
import torchvision
import torch

class ImagePublisher(Node):

    capture_device = 0
    capture_fps = 30
    capture_width = 640
    capture_height = 480
    width = 224
    height = 224

    def __init__(self):
        super().__init__('line_follower')

        self.model = torchvision.models.resnet18(pretrained=False)
        self.model.fc = torch.nn.Linear(512, 2)
        self.model.load_state_dict(torch.load('/workspace/src/ai_race/ai_race/models/road_following_model30a.pth'))
        self.get_logger().info(f'1')
        self.device = torch.device('cuda')
        self.get_logger().info(f'2')
        self.model = self.model.to(self.device)
        self.get_logger().info(f'3')
        self.model = self.model.eval().half()
        self.get_logger().info(f'4')

        self.bridge = cv_bridge.CvBridge()
        self.cmd_vel_pub = self.create_publisher(Twist, 'velocity', 10)
        self.image_pub = self.create_publisher(
            Image, 'video_frames_line_follower', 10)
        self.image_sub = self.create_subscription(
            Image,
            'video_frames',
            self.image_callback,
            10)

        self.get_logger().info(f'5')
        self.twist = Twist()

    def image_callback(self, msg):
        self.get_logger().info(f'6')
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        output = self.model(image).detach().cpu().numpy().flatten()
        self.get_logger().info(output)
        steering = float(output[0])

        velocity = Twist()
        velocity.linear.x = 0.2
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0

        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = steering  

        self.cmd_vel_pub.publish(velocity)

        #self.image_pub.publish(self.bridge.cv2_to_imgmsg(mask))


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)

    image_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
