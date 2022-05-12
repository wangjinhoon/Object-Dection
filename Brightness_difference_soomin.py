import cv2, time
import numpy as np

Width = 640
Height = 480 
cap = cv2.VideoCapture("../label_img4.avi")

_, frame = cap.read()
while not frame.size == (Width*Height*3):
    _, frame = cap.read()
    continue

threshold_60 = 60
threshold_100 = 100

width_640 = 640
scan_width_200, scan_height_20 = 200, 50
lmid, rmid_440 = scan_width_200, width_640 - scan_width_200

area_width, area_height = 30, 30

vertical = 320
row_begin_5 = (scan_height_20 - area_height) // 2
row_end_15 = row_begin_5 + area_height
pixel_threshold = 0.4 * area_width * area_height

def Brightness_difference(frame):

    roi = frame[vertical:vertical + scan_height_20, :]
    #frame = cv2.rectangle(frame, (0, vertical), (width_640 - 1, vertical + scan_height_20), (255, 0, 0), 3)
    # hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # lbound = np.array([0, 0, threshold_100], dtype=np.uint8)
    # ubound = np.array([131, 255, 255], dtype=np.uint8)
    ##ubound = np.array([150, 150, 150], dtype=np.uint8)
    
    # bin = cv2.inRange (hsv, lbound, ubound)

    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    m = cv2.mean(gray)[0]
    gray = cv2.add(gray, 120 - m)
    gray = cv2.GaussianBlur(gray,(5, 5), 3)

    _, reverse = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
    bin = (255 - reverse)[vertical:vertical+scan_height_20, :]
    #bin = cv2.Canny(gray, 10, 200)

    frame = cv2.rectangle(frame, (0, vertical), (width_640 - 1, vertical + scan_height_20), (255, 0, 0), 3)

    view = cv2.cvtColor (bin, cv2.COLOR_GRAY2BGR)

    left, right = -1, -1

    for l in range(area_width, lmid):
        area = bin[row_begin_5:row_end_15, l - area_width:l]
        if cv2.countNonZero(area) > pixel_threshold:
            left = l
            break
    for r in range (width_640 - area_width, rmid_440, -1):
        area = bin[row_begin_5:row_end_15, r:r + area_width] 
        if cv2.countNonZero(area) > pixel_threshold:
            right = r
            break
    if left != -1:
        lsquare = cv2.rectangle(view,
            (left - area_width, row_begin_5),
            (left, row_end_15),
            (0, 255, 0), 3)
    else:
        print("Lost left line")
    if right != -1:
        rsquare = cv2.rectangle(view,
            (right, row_begin_5),
            (right + area_width, row_end_15),
            (0, 255, 0), 3)
    else:
        print("Lost right line")

    # cv2.imshow("gray", gray)
    # cv2.imshow("bin", bin)
    cv2.imshow("frame", frame)
    cv2.imshow("view", view)

    center = (right + left)/2
    shift = center - 320


while cap.isOpened():
    
    _, frame = cap.read()
    
    # image = calibrate_image(frame)
    # image = frame
    # warp_img, M, Minv = warp_image(image, warp_src, warp_dist, (warp_img_w, warp_img_h))
    # left_fit, right_fit = warp_process_image(warp_img)
    # lane_img = draw_lane(image, warp_img, Minv, left_fit, right_fit)
    # cv2.imshow(window_title, lane_img)

    ## color mean ##
    # m1, m2, m3 = cv2.mean(frame)[0], cv2.mean(frame)[1], cv2.mean(frame)[2]
    # dst2 = cv2.add(frame ,(100 - m1, 100 - m2, 100 - m3, 0))
    Brightness_difference(frame)

    cv2.waitKey(0)
    
    # if cv2.waitKey(1) & 0XFF == 27:
    #     break