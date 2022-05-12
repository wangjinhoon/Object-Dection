#!/usr/bin/env python
import cv2, time
import numpy as np
import rospy, math, os, rospkg
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# path = rospkg. RosPack().get_path('line_drive')+"/src/track1.avi"
# cap = cv2.VideoCapture(path)

bridge = CVBridge()
cv_image = np.empty(shape=[0])

threshold_60 = 60
threshold_100 = 100

width_640 = 640
scan_width_200, scan_height_20 = 200, 20
Imid_200, rmid_440 = scan_width_200, width_640 - scan_width_200
area_width_20, area_height_10 = 20, 10

vertical_430 = 430
row_begin_5 = (scan_height_20 - area_height_10) // 2
row_end_15 = row_begin_5 + area_height_10
pixel_threshold_160 = 0.8 * area_width_20 * area_height_10


def img_callback(img_data):
    global bridge
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")

motor_control = xycar_motor()
def pub_motor(angle, speed):
    global pub
    global motor_control
    
    motor_control.angle = angle
    motor_control.speed = speed
    
    pub.publish(motor_control)

def start():
    global pub
    global cv_image

    rospy.init_node('line_follow')
    rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rate = rospy. Rate(20)
    while not rospy.is_shutdown ():
        while not cv_image.size == (640*480*3):
            continue
        frame = cv_image

        if cv2.waitKey(1) & 0XFF == 27:
            break

        roi = frame[vertical_430:vertical_430 + scan_height_20, :]
        frame = cv2.rectangle(frame, (0, vertical_430), (width_640 - 1, vertical_430 + scan_height_20), (255, 0, 0), 3)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lbound = np.array([0, 0, threshold_100], dtype=np.uint8)
        ubound = np.array([131, 255, 255], dtype=np.uint8)
        
        bin = cv2.inRange (hsv, lbound, ubound)
        view = cv2.cvtColor (bin, cv2.COLOR_GRAY2BGR)

        left, right = -1, -1

        for l in range(area_width_20, Imid_200):
            area = bin[row_begin_5:row_end_15, l - area_width_20:l]
            if cv2.countNonZero(area) > pixel_threshold_160:
                left = l
                break
        for r in range (width_640 - area_width_20, rmid_440, -1):
            area = bin[row_begin_5:row_end_15, r:r + area_width_20] 
            if cv2.countNonZero(area) > pixel_threshold_160:
                right = r
                break

    if left != -1:
        lsquare = cv2.rectangle(view,
            (left - area_width_20, row_begin_5),
            (left, row_end_15),
            (0, 255, 0), 3)
    else:
        print("Lost left line")
    if right != -1:
        rsquare = cv2.rectangle(view,
            (right, row_begin_5),
            (right + area_width_20, row_end_15),
            (0, 255, 0), 3)
    else:
        print("Lost right line")

    cv2.imshow("origin", frame)
    cv2.imshow("view", view)


center = (right + left)/2
shift = center - 320
Angle = shift/3
if Angle < -50:
    Angle = -50
if Angle > 50:
    Angle = -50
Speed = 20
pub_motor(Angle, Speed)

cap.release()
cv2. destroyAllwindows ()
if __name__ == '__main__':
    start()




