import cv2
from cv2 import waitKey
from cv2 import threshold

def traffic_light(gray, ract): # ract = (x1, y1, x2, y2)
    
    darkness = -100
    threshold_min = 70
    threshold_max = 255

    gray_dark = cv2.add(gray, darkness)

    _, gray_the = cv2.threshold(gray_dark, threshold_min, threshold_max, cv2.THRESH_BINARY)
    bin = gray_the[ract[1]:ract[3], ract[0]:ract[2]]

    count = 0
    for y in range(0, ract[3]-ract[1]):
        if bin[y,(ract[3]-ract[0])//2] == 255:
            count += 1
        if count >= 30:
            if y < (ract[3]-ract[1])//3:
                print("rad_light")
            elif y < (ract[3]-ract[1])//3 * 2:
                print("yello_light")
            else:
                print("green_light")
            break
    
    cv2.imshow("bin", bin)

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        _, frame = cap.read()

        frame = cv2.resize(frame, dsize=(640, 480))

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ract = (190, 80, 450, 410)
        traffic_light(gray, ract)

        cv2.imshow("frame", frame)

        cv2.waitKey(1)