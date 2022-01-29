from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
import numpy
import cv_bridge
import rclpy
import cv2
from rclpy.node import Node


class ImagePublisher(Node):

    capture_device = 0
    capture_fps = 30
    capture_width = 640
    capture_height = 480
    width = 224
    height = 224

    def __init__(self):
        super().__init__('line_follower')

        self.bridge = cv_bridge.CvBridge()
        self.cmd_vel_pub = self.create_publisher(Twist, 'velocity', 10)
        self.image_pub = self.create_publisher(
            Image, 'video_frames_line_follower', 10)
        self.image_sub = self.create_subscription(
            Image,
            'video_frames',
            self.image_callback,
            10)
        self.twist = Twist()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        #self.image_pub.publish(self.bridge.cv2_to_imgmsg(mask))


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)

    image_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
