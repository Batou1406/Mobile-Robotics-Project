import cv2
import numpy as np

def souris(event, x, y, flags, param):
    global red, green, blue, low_RGB, high_RGB
    if event==cv2.EVENT_LBUTTONDBLCLK:
        blue=frame[y, x][0]
        green=frame[y, x][1]
        red=frame[y, x][2]
        low_RGB=np.array([blue-20,green-20,red-20])
        high_RGB=np.array([blue+20,green+20,red+20])


red, green, blue = 100,100,100
low_RGB=np.zeros(3)
high_RGB=np.zeros(3)
frame=None

def calibration():
    VideoCap=cv2.VideoCapture(0)
    cv2.namedWindow('Camera')
    cv2.setMouseCallback('Camera', souris)
    global frame

    while(True):
        ret, frame=VideoCap.read()
        map_hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(frame, low_RGB, high_RGB)

        cv2.putText(frame, "red: {:d}  green:{:d}  blue{:d}     G for goal, R for robot".
                    format(red, green, blue), (5, 20), cv2.FONT_HERSHEY_PLAIN, 1,
                     (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow('Camera', frame)
        if mask is not None:
            cv2.imshow('mask', mask)

        key=cv2.waitKey(1)&0xFF
        if key==ord('q'):
            VideoCap.release()
            cv2.destroyAllWindows()
            break
        if key==ord('g'):
            blueG, greenG, redG=blue.copy(), green.copy(), red.copy()
            print(blueG, greenG, redG)
        if key==ord('r'):
            blueR, greenR, redR=blue.copy(), green.copy(), red.copy()
            print(blueR, greenR, redR)

    return blueG, greenG, redG, blueR, greenR, redR
