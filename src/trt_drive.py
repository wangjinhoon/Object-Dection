#!/usr/bin/env python2

import rospy, serial, time
from xycar_msgs.msg import xycar_motor
from yolov3_trt.msg import BoundingBoxes, BoundingBox
from Hough_liner import HoughLiner


def stop(cls):
    cls.controller.stop()
    pass


def left(cls):
    pass


def right(cls):
    pass


liner = HoughLiner("Hough_Liner", commands=[stop, left, right])
liner.run()




