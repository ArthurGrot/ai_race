import rclpy
import cv2
import threading
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from threading import Thread, Event
import torch

event = Event()
image_subscriber_yolo = None


class ImageSubscriberYolo(Node):
    def __init__(self):
        super().__init__('image_subscriber_yolo')

        self.model = torch.hub.load('yolov5/', 'custom', path='/home/jetson/AD_AIRACE_YOLO/yolov5/runs/train/exp19/weights/best.pt', source='local') # local repo
        self.model.cuda()

        #example for subscription 
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.listener_callback,
            10)
        self.subscription
        #self.i = 0
        self.br = CvBridge()
        self.frame = None

    def listener_callback(self, data):
        cv_image = self.br.imgmsg_to_cv2(data, desired_encoding="passthrough")

        self.frame = cv2.imencode(".jpg", cv_image)[1].tobytes()
        self.res = self.model(self.frame,size=640)
        self.get_logger().info(f'Detected: {self.res.pandas().xyxy[0]}')

        event.set()

def get_frame():
    event.wait()
    event.clear()
    return image_subscriber_yolo.frame




def gen():
    while True:
        frame = get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')



def runApp():
    rclpy.init()
    global image_subscriber_yolo
    image_subscriber_yolo = ImageSubscriberYolo()

    rclpy.spin(image_subscriber_yolo)
    image_subscriber_yolo.destroy_node()
    rclpy.shutdown()


def main(args=None):
    t1 = threading.Thread(target=runApp).start()


if __name__ == '__main__':
    main()
