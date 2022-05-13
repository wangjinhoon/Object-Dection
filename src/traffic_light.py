import cv2
from cv2 import waitKey
from cv2 import threshold

def traffic_light(gray, ract): # ract = (minx, miny, width, height)
    
    darkness = -100
    threshold_min = 70
    threshold_max = 255
    x1, y1, x2, y2 = ract[0], ract[1], ract[0]+ract[2], ract[1] + ract[3]

    gray_dark = cv2.add(gray, darkness)

    _, gray_the = cv2.threshold(gray_dark, threshold_min, threshold_max, cv2.THRESH_BINARY)
    bin = gray_the[y1:y2, x1:x2]

    cv2.imshow("bin", bin)
    countr = 0
    county = 0
    countg = 1
    for y in range(0, (y2-y1)//3):
        for x in range(0, x2-x1):
            if bin[y,x] == 255:
                countr += 1
    for y in range((y2-y1)//3, (y2-y1)//3 * 2):
        for x in range(0, x2-x1):
            if bin[y,x] == 255:
                county += 1
    for y in range((y2-y1)//3 * 2, y2-y1):
        for x in range(0, x2-x1):
            if bin[y,x] == 255:
                countg += 1
    M = max(countr, county, countg)
    if(M == countr) : return 5
    elif(M == county) : return 6
    else : return 7