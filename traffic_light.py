import cv2
from cv2 import waitKey
from cv2 import threshold

def traffic_light(gray, ract): # ract = (minx, miny, width, height)
    
    w_ratio = 640/416.
    h_ratio = 480/416.

    darkness = -100
    threshold_min = 70
    threshold_max = 255

    x1, y1, x2, y2 = int(ract[0] * w_ratio), int(ract[1] * h_ratio), int((ract[0]+ract[2]) * w_ratio), int((ract[1] + ract[3]) * h_ratio)

    gray_dark = cv2.add(gray, darkness)

    _, gray_the = cv2.threshold(gray_dark, threshold_min, threshold_max, cv2.THRESH_BINARY)
    bin_origin = gray_the[y1:y2, x1:x2]
    bin = gray_the[y1+int((y2-y1)*0.1):y2-int((y2-y1)*0.1), x1+int((x2-x1)*0.2):x2-int((x2-x1)*0.2)]

    cv2.imshow("bin", bin)
    cv2.imshow("bin_origin", bin_origin)
    countr = 0
    county = 0
    countg = 1
    
    w = (x2-x1) - int((x2-x1)*0.4)
    h = (y2-y1) - int((y2-y1)*0.2)
    
    for y in range(0, h//3):
        for x in range(0, w):
            if bin[y,x] == 255:
                countr += 1
    for y in range(h//3, h//3 * 2):
        for x in range(0, w):
            if bin[y,x] == 255:
                county += 1
    for y in range(h//3 * 2, h):
        for x in range(0, w):
            if bin[y,x] == 255:
                countg += 1
    M = max(countr, county, countg)
    if(M == countr) : return 5
    elif(M == county) : return 6
    else : return 7

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        _, frame = cap.read()

        frame = cv2.resize(frame, dsize=(640, 480))

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ract = (190, 80, 200, 410-80)

        tl = traffic_light(gray, ract)

        if tl == 5:
            print("rad_light")
        elif tl == 6:
            print("yello_light")
        else:
            print("green_light")


        cv2.imshow("frame", frame)
        cv2.waitKey(1)