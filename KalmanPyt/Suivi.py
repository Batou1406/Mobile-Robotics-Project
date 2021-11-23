import cv2
from Detector import detect_inrange, detect_visage
from KalmanFilter import KalmanFilter
import numpy as np

def souris(event, x, y, flags, param):
    global lo, hi, color
    if event==cv2.EVENT_LBUTTONDBLCLK:
        color=frame[y, x][0]
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
KF=KalmanFilter(0.1, [0, 0])

VideoCap=cv2.VideoCapture(1)
cv2.namedWindow('Camera')
cv2.setMouseCallback('Camera', souris)

while(True):
    ret, frame=VideoCap.read()

    points, mask=detect_inrange(frame, 800, color, S, V)
    #points, mask=detect_visage(frame)

    etat=KF.predict().astype(np.int32)
    cv2.circle(frame, (int(etat[0]), int(etat[1])), 2, (0, 255, 0), 5)
    cv2.arrowedLine(frame, (int(etat[0]),int(etat[1])),(int(etat[0]+etat[2]),
                    int(etat[1]+etat[3])),color=(0, 255, 0),thickness=3,
                    tipLength=0.2)

    if (len(points)>0):
        cv2.circle(frame, (points[0][0], points[0][1]), 10, (0, 0, 255), 2)
        z = np.matrix([[points[0][0]],[ points[0][1]]])
        KF.update(z,'image')
    #else:
        #alpha = np.arctan2(etat[2], etat[3])+1
        #z = np.matrix([[int(20*np.cos(alpha))], [int(20*np.sin(alpha))]])
        #KF.update(z, 'speed')
        #KF.updateWheels(z)

    cv2.putText(frame, "[Souris]Couleur: {:d}    [o|l] S:{:d}    [p|m] V{:d}".
                format(color, S, V), (5, 20), cv2.FONT_HERSHEY_PLAIN, 1,
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
