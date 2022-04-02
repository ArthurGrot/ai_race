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
    
    class DetectionDetails():
        # Dataobject for each detection
        def __init__(self, name, px, py, width, distance):
            self.name = name
            self.px = px
            self.py = py 
            self.width = width
            self.distance = distance
        
        def ToString(self):
            return f"Detected: {self.name} at ({self.px},{self.py}) {self.distance}cm away"


    def distance_finder(focalLength, realObjectWidth, objectWidthInFrame):
        distance = ((realObjectWidth * focalLength)/objectWidthInFrame)*100
        return distance


    def find_width(self, image):
        object_width = 0
        res = self.model(image,size=640)
        res.render()  
        coords = res.pandas().xyxy[0]
        for obj in coords.itertuples(index = True, name ='Pandas'):
            xmin = getattr(obj, "xmin")
            xmax = getattr(obj, "xmax")

            object_width = ((xmax - xmin)/640)
        return object_width


    def __init__(self):
        super().__init__('image_subscriber_yolo')
        # '/workspace/src/ai_race/ai_race/models/road_following_model30a.pth'
        self.model = torch.hub.load('/workspace/src/ai_race/ai_race/yolov5/', 'custom', path='/workspace/src/ai_race/ai_race/models/yolov5Detection.pt', source='local') # local repo
        self.model.cuda()
        # parameters for distance estimation
        self.focal_length = 0.315

            # reference images
        michael_ref = cv2.imread("PATH TO MICHAEL REF IMAGE")
        nicole_ref = cv2.imread("PATH TO NICOLE REF IMAGE")
        lena_ref = cv2.imread("PATH TO LENA REF IMAGE")
        anna_ref = cv2.imread("PATH TO ANNA REF IMAGE")
            # reference results
        michael_res = self.model(michael_ref,size=640)
        nicole_res = self.model(nicole_ref,size=640)
        lena_res = self.model(lena_ref,size=640)
        anna_res = self.model(anna_ref,size=640)
            # reference widths

        obj = michael_res.pandas().xyxy[0].at(0)
        self.michael = self.DetectionDetails("Michael",)


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
        self.cv_image = self.br.imgmsg_to_cv2(data, desired_encoding="passthrough")

        self.res = self.model(self.cv_image,size=640)
        # self.get_logger().info(f'Detected: {self.res.pandas().xyxy[0]}')
        # differentiate between speed and playmobil
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
    runApp()


if __name__ == '__main__':
    main()
