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
from datetime import datetime
from std_msgs.msg import Bool

mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
std = torch.Tensor([0.229, 0.224, 0.225]).cuda()

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('line_follower')

        self.model = torchvision.models.resnet18(pretrained=False)
        self.model.fc = torch.nn.Linear(512, 2)
        self.model.load_state_dict(torch.load('/workspace/src/ai_race/ai_race/models/road_following_model_3_50a.pth'))
        self.device = torch.device('cuda')
        self.model = self.model.to(self.device)
        self.model = self.model.eval().half()

        self.bridge = cv_bridge.CvBridge()
        self.cmd_vel_pub = self.create_publisher(Twist, 'steering_line', 1)
        self.image_sub = self.create_subscription(Image, 'video_frames_224', self.image_callback, 1)
       
        self.twist = Twist()

        self.get_logger().info(f'Init finished')
        
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        torch_ex_float_tensor = self.preprocess(image).half()
        output = self.model(torch_ex_float_tensor).detach().cpu().numpy().flatten()
            
        velocity = Twist()
        # velocity.linear.x = float(-0.4)
        velocity.angular.z = float(output[0]) * -1.0  

        self.cmd_vel_pub.publish(velocity)
        self.get_logger().info(f'Position estimated')

    def preprocess(self, image):
        image = PIL.Image.fromarray(image)
        image = transforms.functional.to_tensor(image).to(self.device)
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
