import time
from abc import abstractmethod
import rospy
from sensor_msgs.msg import Image
from yolov3_trt_ros.msg import BoundingBox, BoundingBoxes
from cv_bridge import CvBridge

from PID import PID
from ego_controller import egoController


class Liner:
    def __init__(self, node_name, commands=None):
        rospy.init_node(node_name)
        self.sub = rospy.Subscriber('/usb_cam/image_raw',
                                    Image, self.callback, queue_size=1)
        if commands:
            self.sub_itrpt = rospy.Subscriber('/yolov3_trt_ros/detections', BoundingBoxes, self.callback_itrpt, queue_size=1)
            self.commands = commands    # functions

        self.bridge = CvBridge()
        self.pid = PID(0.5, 0.0005, 0.00005)
        self.controller = egoController()
        self.controller.stop()
        time.sleep(3)

    def imgmsg2numpy(self, msg):
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    @abstractmethod
    def callback(self, msg):
        pass

    @abstractmethod
    def callback_itrpt(self, msg):
        pass

    @staticmethod
    def run():
        rospy.spin()