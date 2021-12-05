import rclpy
import math
import requests
import threading
import Adafruit_SSD1306
import time
import PIL.Image
import PIL.ImageFont
import PIL.ImageDraw

from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult
from .utils import ip_address, power_mode, power_usage, cpu_usage, gpu_usage, memory_usage, disk_usage

display_service_off_url = "http://127.0.0.1:8000/stats/off"
display_service_on_url = "http://127.0.0.1:8000/stats/on"
display_service_text_url = "http://127.0.0.1:8000/text/"


class Display(Node):
    
    def __init__(self):
        super().__init__('display')
        
        self.sub = self.create_subscription(String, 'display', self.display_listener, 10)
        rs = requests.get(display_service_off_url)

        self.display = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1) 
        self.display.begin()
        self.display.clear()
        self.display.display()
        self.font = PIL.ImageFont.load_default()
        self.image = PIL.Image.new('1', (self.display.width, self.display.height))
        self.draw = PIL.ImageDraw.Draw(self.image)
        self.draw.rectangle((0, 0, self.image.width, self.image.height), outline=0, fill=0)
        self.stats_enabled = False
        self.stats_thread = None
        self.stats_interval = 1.0
        self.enable_stats()

    def display_listener(self, msg):
        None
    
    def _run_display_stats(self):
        number = 0
        while self.stats_enabled:
            # self.draw.rectangle((0, 0, self.image.width, self.image.height), outline=0, fill=0)

            # # set IP address
            # top = -2
            # if ip_address('eth0') is not None:
            #     self.draw.text((4, top), 'IP: ' + str(ip_address('eth0')), font=self.font, fill=255)
            # elif ip_address('wlan0') is not None:
            #     self.draw.text((4, top), 'IP: ' + str(ip_address('wlan0')), font=self.font, fill=255)
            # else:
            #     self.draw.text((4, top), 'IP: not available')

            # top = 6
            
            # # set stats headers
            # top = 14
            # offset = 3 * 8
            # headers = ['TST', 'CPU', 'GPU', 'RAM', 'DSK']
            # for i, header in enumerate(headers):
            #     self.draw.text((i * offset + 4, top), header, font=self.font, fill=255)

            # # set stats fields
            # top = 22
            # power_watts = '%.1f' % power_usage()
            # gpu_percent = '%02d%%' % int(round(gpu_usage() * 100.0, 1))
            # cpu_percent = '%02d%%' % int(round(cpu_usage() * 100.0, 1))
            # ram_percent = '%02d%%' % int(round(memory_usage() * 100.0, 1))
            # disk_percent = '%02d%%' % int(round(disk_usage() * 100.0, 1))
            
            # entries = [power_watts, cpu_percent, gpu_percent, ram_percent, disk_percent]
            # for i, entry in enumerate(entries):
            #     self.draw.text((i * offset + 4, top), entry, font=self.font, fill=255)

            # self.display.image(self.image)
            # self.display.display()

            rs = requests.get(display_service_text_url + f"test %0A {number}")
            number += 1

            time.sleep(self.stats_interval)

    def enable_stats(self):
        # start stats display thread
        if not self.stats_enabled:
            self.stats_enabled = True
            self.stats_thread = threading.Thread(target=self._run_display_stats)
            self.stats_thread.start()

    def disable_stats(self):
        self.stats_enabled = False
        if self.stats_thread is not None:
            self.stats_thread.join()
        self.draw.rectangle((0, 0, self.image.width, self.image.height), outline=0, fill=0)
        self.display.image(self.image)
        self.display.display()

    def destroy_node(self):
        self.get_logger().info(f"shutting down...")
        self.disable_stats()
        rs = requests.get(display_service_on_url)


def main(args=None):
    rclpy.init(args=args)

    node = Display()
    node.get_logger().info("listening for display messages...")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("node stopped by keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()
     
if __name__ == '__main__':
    main()
    