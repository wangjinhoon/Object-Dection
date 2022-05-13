#!/usr/bin/env python

import rospy
from xycar_msgs.msg import xycar_motor


class egoController:
    def __init__(self):
        self.pub = rospy.Publisher('xycar_motor', xycar_motor)

    def stop(self):
        msg = xycar_motor()
        msg.speed = 0
        msg.angle = 0
        self.pub.publish(msg)

    def go(self, angle):
        msg = xycar_motor()
        msg.speed = 5
        msg.angle = angle
        self.pub.publish(msg)
