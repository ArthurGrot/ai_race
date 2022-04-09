import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from datetime import datetime

#node definition with "(Node)" as parameter
class ImagePublisher(Node):

    capture_device = 0
    capture_fps = 5
    capture_width = 640
    capture_height = 480
    width = 480
    height = 480

    #ctor
    def __init__(self):
        #call super method necessary
        super().__init__('image_publisher')

        #example for publisher
        self.publisher_224 = self.create_publisher(Image, 'video_frames_224', 5)
        self.publisher_480 = self.create_publisher(Image, 'video_frames_480', 5)

        self.cap = cv2.VideoCapture(self._gst_str(), cv2.CAP_GSTREAMER)
        self.br = CvBridge()

        while self.cap.isOpened():
            ret, frame = self.cap.read()

            if ret == True:
                #example for publishing and converting image
                self.publisher_480.publish(self.br.cv2_to_imgmsg(frame))
                frame_224 = self.resize224(frame)
                self.publisher_224.publish(self.br.cv2_to_imgmsg(frame_224))

    def _gst_str(self):
        return 'nvarguscamerasrc sensor-id=%d ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! nvvidconv ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink' % (
            self.capture_device, self.capture_width, self.capture_height, self.capture_fps, self.width, self.height)

    def resize224(self,img):
        dimension = (224,224)
        return cv2.resize(img,dimension,interpolation = cv2.INTER_LINEAR) ## bilinear interpolation

#main method important
def main(args=None):
    #initialize node library 
    rclpy.init(args=args)
    #create instance of node
    image_publisher = ImagePublisher()
    #start node as infinite loop until interupt
    rclpy.spin(image_publisher)

    #disposing
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
