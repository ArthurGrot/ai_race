import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
 

 
class ImagePublisher(Node):

  capture_device = 0
  capture_fps = 30
  capture_width = 640
  capture_height = 480
  width = 224
  height = 224

  def __init__(self):
    super().__init__('image_publisher')
    
    self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
      
    timer_period = 0.1
      
    self.timer = self.create_timer(timer_period, self.timer_callback)      
    self.cap = cv2.VideoCapture(self._gst_str(), cv2.CAP_GSTREAMER)
    self.br = CvBridge()
  
  def _gst_str(self):
    return 'nvarguscamerasrc sensor-id=%d ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! nvvidconv ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink' % (
        self.capture_device, self.capture_width, self.capture_height, self.capture_fps, self.width, self.height)

  def timer_callback(self):

    ret, frame = self.cap.read()
          
    if ret == True:
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
 
    self.get_logger().info(f'Publishing video frame {ret}')
  
def main(args=None):
  rclpy.init(args=args)
  image_publisher = ImagePublisher()
  rclpy.spin(image_publisher)
  
  image_publisher.destroy_node()
  
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()