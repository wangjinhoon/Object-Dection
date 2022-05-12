#!/usr/bin/env python2

import rospy, serial, time
from xycar_msgs.msg import xycar_motor
from Hough_liner import HoughLiner


def stop(cls):
    cls.controller.go(0)
    pass


def left(cls):
    cls.controller.go(-50)
    pass


def right(cls):
    cls.controller.go(50)


liner = HoughLiner("Hough_Liner", commands=[left, right, stop])
liner.run()




