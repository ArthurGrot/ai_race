import cv2
import time
import os

class ImageCollector():
    capture_device = 0
    capture_fps = 2
    capture_width = 640
    capture_height = 480
    width = 480
    height = 480

    def __init__(self):
        self.cap = cv2.VideoCapture(self._gst_str(), cv2.CAP_GSTREAMER)
        path = '/home/jetson/ai_race/yolo_data_collection/collected_images'
        while self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret == True:
                    x = str(time.time())
                    print("new Image " + x)
                    cv2.imwrite(os.path.join(path, x+".png"), frame)
    def _gst_str(self):
        return 'nvarguscamerasrc sensor-id=%d ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! nvvidconv ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink' % (self.capture_device, self.capture_width, self.capture_height, self.capture_fps, self.width, self.height)

def main(args=None):
    collector = ImageCollector()


main()