import rclpy
import cv2
import threading
from flask import Flask, render_template, Response
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from threading import Thread, Event

app = Flask(__name__)
event = Event()
image_subscriber = None


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('flask_server')

        #example for subscription 
        self.subscription = self.create_subscription(
            Image,
            'video_frames_224',
            self.listener_callback,
            10)
        self.subscription
        #self.i = 0
        self.br = CvBridge()
        self.frame = None

    def listener_callback(self, data):
        cv_image = self.br.imgmsg_to_cv2(data, desired_encoding="passthrough")
        self.frame = cv2.imencode(".jpg", cv_image)[1].tobytes()
        event.set()


def get_frame():
    event.wait()
    event.clear()
    return image_subscriber.frame


@app.route('/')
def index():
    return render_template('index.html')


def gen():
    while True:
        frame = get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


def runApp():
    rclpy.init()
    global image_subscriber
    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


def runFlask():
    app.run(host='0.0.0.0', port=8080)


def main(args=None):

    t1 = threading.Thread(target=runApp).start()
    t2 = threading.Thread(target=runFlask).start()


if __name__ == '__main__':
    main()
