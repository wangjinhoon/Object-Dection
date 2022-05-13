import cv2

cap = cv2.VideoCapture(0)

while cap.isOpened():
    
    _, frame = cap.read()

    frame = cv2.resize(frame, dsize=(640, 480))

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    gray_dark = cv2.add(gray, -100)

    x1, y1, x2, y2 = 190, 80, 450, 410
    
    _, gray_the = cv2.threshold(gray_dark, 70, 255, cv2.THRESH_BINARY)
    gray_the = gray_the[y1:y2, x1:x2]

    cv2.line(frame, ((x2-x1)//2 + x1,0), ((x2-x1)//2 + x1, 480), (255,0,255), 2)

    cv2.rectangle(frame, (x1, y1), (x2, y2),(255,255,255), 3)

    count = 0
    for y in range(0, y2-y1):
        if gray_the[y,(y2-x1)//2] == 255:
            count += 1
        if count >= 30:
            color = (255,0,255)
            if y < (y2-y1)//3:
                color = (0,0,255)
            elif y < (y2-y1)//3 * 2:
                color = (200,50,100)

            cv2.circle(frame, ((x2-x1)//2 + x1, y+y1), 10, color, -1)
            break

    cv2.imshow("frame", frame)
    cv2.imshow("gray", gray)
    cv2.imshow("gray_mean", gray_mean)
    cv2.imshow("gray_the", gray_the)

    cv2.waitKey(1)
