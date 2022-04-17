import rclpy
import cv2
import threading
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from threading import Thread, Event
import torch
import math

event = Event()
image_subscriber_yolo = None


class ImageSubscriberYolo(Node):
    
    
    def distance_finder(focalLength, realObjectWidth, objectWidthInFrame):
        distance = ((realObjectWidth * focalLength)/objectWidthInFrame)*100
        return distance


    def find_width(self, image):
        object_width = 0
        res = self.model(image,size=480)
        res.render()  
        coords = res.pandas().xyxy[0]
        for obj in coords.itertuples(index = True, name ='Pandas'):
            xmin = getattr(obj, "xmin")
            xmax = getattr(obj, "xmax")

            object_width = ((xmax - xmin)/480)
        return object_width


    def __init__(self):
        super().__init__('image_subscriber_yolo')
        # '/workspace/src/ai_race/ai_race/models/road_following_model30a.pth'
        
        self.model = torch.hub.load('/workspace/src/ai_race/ai_race/yolov5/', 'custom', path='/workspace/src/ai_race/ai_race/models/best.pt', source='local') # local repo
        self.model.cuda()
        
        # parameters for distance estimation
        self.focal_length = 0.315
        self.michael_count = 0

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
        self.motor_twist = Twist()

        # indecees
        # (name, xmin, ymin, object_width, object_height, distance, conf)
        for obj in yolo_data:
            if obj[6] > 0.8: # confidence greater than ...
                # Speed params
                if obj[0] == "Speed 30":    # speed 30 detected
                    self.motor_twist.angular.x = 0.4
                elif obj[0] == "Speed 50":   # speed 50 detected
                    self.motor_twist.angular.x = 0.55
                elif obj[0] == "Michael":
                    self.motor_twist = self.figurine_detected(self, obj)
                    self.michael_count += 1
                self.get_logger().info(f"YOLO | Detected: {obj[0]} with Conf {obj[6]} at ({obj[1]},{obj[2]}) {obj[5]}cm away")
        if(self.motor_twist != None):
            self.speed_pub.publish(self.motor_twist)
        # differentiate between speed and playmobil

    def figurine_detected(self, obj): 
        """ 
        does the required calculations for whether or not the vehicle has to act, depending on where the object is in the image (center of mass)
        """
        # calculate center of mass (x,y)
        center_of_mass = (round((obj[1]+obj[3])/2), round((obj[2]+obj[4])/2))
        x_quadrant = math.floor(center_of_mass[0]/160)
        y_quadrant = math.floor(center_of_mass[1]/160)
        twist = Twist()
        
        # decide based upon the segmented zones in the image
        # zones =   0-159,0-159   | 159-320, 0-159   | 321-480, 0-159 
        #           0-159,159-320 | 159-320, 159-320 | 321-480, 159-320
        #           0-159,321-480 | 159-320, 321-480 | 321-480, 321-480 
        if(y_quadrant == 0 & x_quadrant == 1):
            # Do nothing yet needs to be seen if necessary
            return
        if(y_quadrant == 1 & x_quadrant == 1):
                twist.angular.z = 1.0
        elif(y_quadrant == 2):
            if(x_quadrant == 0):
                twist.angular.z = 0.3
            elif(x_quadrant == 1):
                com = center_of_mass[0]/160
                if(com > 1.5):
                    twist.angular.z = - 0.7
                else:
                    twist.angular.z = 0.7
            elif(x_quadrant == 2):
                twist.angular.z = -0.3
                
        if(self.michael_count >= 5):
            return twist
        else:
            return None

    
    def get_data(self, img):
        """ returns a tuple of size seven containing \n
        (name, xmin, ymin, object_width, object_height, distance, conf)
        """
        res = self.model(self.cv_image,size=480)
        data = res.pandas().xyxy[0]
        objects_in_frame = []
        for obj in data.itertuples(index = True, name ='Pandas'):
            xmin = getattr(obj, "xmin")
            xmax = getattr(obj, "xmax")
            ymin = getattr(obj, "ymin")
            ymax = getattr(obj, "ymax")
            
            name = getattr(obj, "name")
            conf = getattr(obj, "confidence")
            object_width = ((xmax - xmin)/480)
            object_height = ((ymax - ymin)/480)

            distance =  0 #self.distance_finder() # add once necessary # turns out that it isnt necessary yet


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
