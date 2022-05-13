import cv2
from cv2 import cvtColor

rad_jpg = cv2.imread("KakaoTalk_20220512_202218137.jpg")
yello_jpg = cv2.imread("KakaoTalk_20220512_202218137_01.jpg")
green_jpg = cv2.imread("KakaoTalk_20220512_202218137_02.jpg")

rad_jpg = cv2.resize(rad_jpg, dsize=(640, 480))
yello_jpg = cv2.resize(yello_jpg, dsize=(640, 480))
green_jpg = cv2.resize(green_jpg, dsize=(640, 480))

rad = cvtColor(rad_jpg, cv2.COLOR_BGR2GRAY)
yello = cvtColor(yello_jpg, cv2.COLOR_BGR2GRAY)
green = cvtColor(green_jpg, cv2.COLOR_BGR2GRAY)


rad = cv2.add(rad, -100)
yello = cv2.add(yello, -100)
green = cv2.add(green, -100)

rx1, ry1, rx2, ry2 = 190, 80, 450, 410
yx1, yy1, yx2, yy2 = 190, 80, 450, 410
bx1, by1, bx2, by2 = 190, 80, 450, 420

_, rad = cv2.threshold(rad, 70, 255, cv2.THRESH_BINARY)
_, yello = cv2.threshold(yello, 70, 255, cv2.THRESH_BINARY)
_, green = cv2.threshold(green, 70, 255, cv2.THRESH_BINARY)

rad = rad[ry1:ry2,rx1:rx2]
yello = yello[yy1:yy2,yx1:yx2]
green = green[by1:by2,bx1:bx2]


count = 0
for r in range(0, ry2-ry1):
    if rad[r,(rx2-rx1)//2] == 255:
        count += 1
    if count >= 30:
        print("rad")
        cv2.circle(rad, ((rx2-rx1)//2, r), 10, 50, -1)
        break

count = 0
for y in range(0, yy2-yy1):
    if yello[y,(yx2-yx1)//2] == 255:
        count += 1
    if count >= 30:
        print("yello")
        cv2.circle(yello, ((yx2-yx1)//2, y), 10, 50, -1)
        break

count = 0
for b in range(0, by2-by1):
    if green[b,(bx2-bx1)//2] == 255:
        count += 1
    if count >= 30:
        print("green")
        cv2.circle(green, ((bx2-bx1)//2, b), 10, 50, -1)
        break

cv2.line(rad, ((rx2-rx1)//2,0), ((rx2-rx1)//2, ry2-ry1), 255, 3)
cv2.line(yello, ((yx2-yx1)//2,0), ((yx2-yx1)//2, yy2-yy1), 255, 3)
cv2.line(green, ((bx2-bx1)//2,0), ((bx2-bx1)//2, by2-by1), 255, 3)

cv2.rectangle(rad_jpg, (rx1, ry1), (rx2, ry2),(255,255,255), 3)
cv2.rectangle(yello_jpg, (yx1, yy1), (yx2, yy2),(255,255,255), 3)
cv2.rectangle(green_jpg, (bx1, by1), (bx2, by2),(255,255,255), 3)


cv2.imshow("rad_jpg", rad_jpg)
cv2.imshow("yello_jpg", yello_jpg)
cv2.imshow("green_jpg", green_jpg)

cv2.imshow("rad", rad)
cv2.imshow("yello", yello)
cv2.imshow("green", green)

cv2.waitKey()

