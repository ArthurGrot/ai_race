import rclpy
import cv2
import threading
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from threading import Thread, Event
import torch

event = Event()
image_subscriber_yolo = None


class ImageSubscriberYolo(Node):
    
    
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
        
        self.model = torch.hub.load('/workspace/src/ai_race/ai_race/yolov5/', 'custom', path='/workspace/src/ai_race/ai_race/models/best.pt', source='local') # local repo
        self.model.cuda()
        
        # parameters for distance estimation
        self.focal_length = 0.315

            # reference images
        #michael_ref = cv2.imread("PATH TO MICHAEL REF IMAGE")
        #nicole_ref = cv2.imread("PATH TO NICOLE REF IMAGE")
        #lena_ref = cv2.imread("PATH TO LENA REF IMAGE")
        #anna_ref = cv2.imread("PATH TO ANNA REF IMAGE")
        #    # reference results
        #michael_res = self.model(michael_ref,size=640)
        #nicole_res = self.model(nicole_ref,size=640)
        #lena_res = self.model(lena_ref,size=640)
        #anna_res = self.model(anna_ref,size=640)
        #    # reference widths
        #
        #obj = michael_res.pandas().xyxy[0].at(0)
        #self.michael = self.DetectionDetails("Michael",)


        #example for subscription 
        self.subscription = self.create_subscription(
            Image,
            'video_frames_480',
            self.listener_callback,
            5)
        self.subscription
        #self.i = 0
        self.br = CvBridge()
        self.frame = None
        # yolo speed publisher
        self.speed_pub = self.create_publisher(Twist, 'speed_yolo', 1)

    


    def listener_callback(self, data):
        self.cv_image = self.br.imgmsg_to_cv2(data, desired_encoding="passthrough")
        yolo_data = self.get_data(self.cv_image)
        motor_twist = Twist()

        log = ""
        # indecees
        # (name, xmin, ymin, object_width, object_height, distance, conf)
        for obj in yolo_data:
            if obj[6] > 0.5: # confidence greater than ...
                # Speed params
                if obj[0] == "Speed 30":    # speed 30 detected
                    motor_twist.angular.x = 0.4
                elif obj[0] == "Speed 50":   # speed 50 detected
                    motor_twist.angular.x = 0.55
                elif obj[0] == "Michael":
                    motor_twist.angular.x = 0.0
                log += f"Speed set to {motor_twist.angular.x}"
                self.get_logger().info(f"YOLO | Detected: {obj[0]} with Conf {obj[6]} at ({obj[1]},{obj[2]}) {obj[5]}cm away")
        
        self.speed_pub.publish(motor_twist)
        self.get_logger().info(f"YOLO | {log}")
        # differentiate between speed and playmobil
    
    def get_data(self, img):
        res = self.model(self.cv_image,size=256)
        data = res.pandas().xyxy[0]
        objects_in_frame = []
        for obj in data.itertuples(index = True, name ='Pandas'):
            xmin = getattr(obj, "xmin")
            xmax = getattr(obj, "xmax")
            ymin = getattr(obj, "ymin")
            ymax = getattr(obj, "ymax")
            
            name = getattr(obj, "name")
            conf = getattr(obj, "confidence")
            object_width = ((xmax - xmin)/640)
            object_height = ((ymax - ymin)/480)

            distance =  0 #self.distance_finder() # add once necessary


            detected_object = (name, xmin, ymin, object_width, object_height, distance, conf)
            objects_in_frame.append(detected_object)

        return objects_in_frame


    def distance_finder(self, realObjectWidth, objectWidthInFrame):
        distance = ((realObjectWidth * self.focal_length)/objectWidthInFrame)*100
        return distance



def runApp():
    rclpy.init()
    global image_subscriber_yolo
    image_subscriber_yolo = ImageSubscriberYolo()

    rclpy.spin(image_subscriber_yolo)
    rclpy.shutdown()


def main(args=None):
    runApp()


if __name__ == '__main__':
    main()
