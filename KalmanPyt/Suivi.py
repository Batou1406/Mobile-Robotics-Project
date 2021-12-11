import cv2
from Detector import detect_inrange, detect_visage
from KalmanFilter import KalmanFilter
import numpy as np

def souris(event, x, y, flags, param):
    global lo, hi, color, red, green, blue
    if event==cv2.EVENT_LBUTTONDBLCLK:
        color=frame[y, x][0]
        blue=frame[y, x][0]
        green=frame[y, x][1]
        red=frame[y, x][2]
    if event==cv2.EVENT_MOUSEWHEEL:
        if flags<0:
            if color>5:
                color-=1
        else:
            if color<250:
                color+=1

color=90
S=50
V=50
red, green, blue = 0,0,0
u=np.matrix([[0],[0],[0]])
x=[0,0,0]
KF=KalmanFilter(x)

VideoCap=cv2.VideoCapture(0)
cv2.namedWindow('Camera')
cv2.setMouseCallback('Camera', souris)

while(True):
    ret, frame=VideoCap.read()

    points, mask=detect_inrange(frame, 800, color, S, V)

    x=KF.predict(u, 0.1).astype(np.int32)

    cv2.circle(frame, (int(x[0]), int(x[1])), 2, (0, 255, 0), 5)
    cv2.arrowedLine(frame, (int(x[0]),int(x[1])),(int(x[0]+30*np.cos(x[2])),
                    int(x[1]+30*np.sin(x[2]))),color=(0, 255, 0),thickness=3,
                    tipLength=0.2)

    if (len(points)>0):
        cv2.circle(frame, (points[0][0], points[0][1]), 10, (0, 0, 255), 2)
        z=np.matrix([[points[0][0]], [points[0][1]], [0]])
        KF.update(z)
        u=np.matrix([[0],[0],[0]])
    else:
        vx=(10*np.cos(x[2]))
        vy=(10*np.cos(x[2]))
        u=np.matrix([[vx],[vy],[1]])


    cv2.putText(frame, "[Souris] blue: {:d}   [o|l] green:{:d}  [p|m] red{:d}".
                format(blue, green, red), (5, 20), cv2.FONT_HERSHEY_PLAIN, 1,
                 (255, 255, 255), 1, cv2.LINE_AA)


    cv2.imshow('Camera', frame)
    if mask is not None:
        cv2.imshow('mask', mask)

    key=cv2.waitKey(1)&0xFF
    if key==ord('q'):
        VideoCap.release()
        cv2.destroyAllWindows()
        break
    if key==ord('p'):
        V=min(255, V+1)
        lo=np.array([color-5, S, V])
    if key==ord('m'):
        V=max(1, V-1)
        lo=np.array([color-5, S, V])
    if key==ord('o'):
        S=min(255, S+1)
        lo=np.array([color-5, S, V])
    if key==ord('l'):
        S=max(1, S-1)
        lo=np.array([color-5, S, V])
