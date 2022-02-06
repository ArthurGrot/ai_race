from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy
import cv_bridge
import rclpy
import cv2
from rclpy.node import Node
import torchvision
import torch
import torchvision.transforms as transforms
import torch.nn.functional as F
import PIL.Image


mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
std = torch.Tensor([0.229, 0.224, 0.225]).cuda()

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
        self.device = torch.device('cuda')
        self.model = self.model.to(self.device)
        self.model = self.model.eval().half()

        self.transform = transforms.ToTensor()

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
        torch_ex_float_tensor = self.preprocess(image).half()
        self.get_logger().info(f'')
        output = self.model(torch_ex_float_tensor).detach().cpu().numpy().flatten()
        steering = float(output[0])

        velocity = Twist()
        velocity.linear.x = -0.8
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0

        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = steering  

        self.cmd_vel_pub.publish(velocity)

        #self.image_pub.publish(self.bridge.cv2_to_imgmsg(mask))

    def preprocess(self, image):
        device = torch.device('cuda')
        image = PIL.Image.fromarray(image)
        image = transforms.functional.to_tensor(image).to(device)
        image.sub_(mean[:, None, None]).div_(std[:, None, None])
        return image[None, ...]


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)

    image_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
