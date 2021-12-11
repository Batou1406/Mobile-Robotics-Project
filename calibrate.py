import cv2
import numpy as np

def souris(event, x, y, flags, param):
    global red, green, blue, low, high, H, S, V,tresh, HSV
    if event==cv2.EVENT_LBUTTONDBLCLK:
        blue=frame[y, x][0]
        green=frame[y, x][1]
        red=frame[y, x][2]
        low=np.array([blue-30,green-30,red-30])
        high=np.array([blue+30,green+30,red+30])
        HSV=False
        tresh=False
    if event==cv2.EVENT_MOUSEWHEEL:
        if flags<0:
            if H>5:
                H-=1
        else:
            if H<250:
                H+=1
        tresh=False
        HSV=True
        low=np.array([H-15,S,V])
        high=np.array([H+15,255,255])


def calibration():
    VideoCap=cv2.VideoCapture(1+cv2.CAP_DSHOW)
    cv2.namedWindow('Camera')
    cv2.setMouseCallback('Camera', souris)
    global frame
    global tresh
    global low
    global high
    global H,S,V
    global treshhold
    global HSV
    global red, green, blue
    frame=None
    tresh = False
    low=np.zeros(3)
    high=np.zeros(3)
    H,S,V = 90,50,50
    treshhold=70
    HSV=False
    red, green, blue=100,100,100

    while(True):
        ret, frame=VideoCap.read()
        image=cv2.blur(frame, (5, 5))
        if(HSV):
            map_hsv=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            mask=cv2.inRange(map_hsv, low, high)
        elif(tresh):
            map_rgb=cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            map_blur=cv2.blur(map_rgb,(10,10))
            map_rgb=cv2.cvtColor(map_blur, cv2.COLOR_BGR2RGB)
            map_bilateral=cv2.bilateralFilter(map_rgb,5,15,15)
            map_gray=cv2.cvtColor(map_bilateral, cv2.COLOR_RGB2GRAY)
            _, map_binary=cv2.threshold(map_gray, treshhold, 255, cv2.THRESH_BINARY_INV)
            mask=map_binary
        else:
            mask=cv2.inRange(frame, low, high)

        if(not tresh):
            points=[]
            elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            elements=sorted(elements, key=lambda x:cv2.contourArea(x), reverse=True)
            surface = 20
            for element in elements:
                if cv2.contourArea(element)>surface:
                    ((x, y), rayon)=cv2.minEnclosingCircle(element)
                    points.append(np.array([int(x), int(y)]))
            if (len(points)>0):
                cv2.circle(frame, (points[0][0], points[0][1]), 10, (0, 0, 255), 2)

        cv2.putText(frame, "red:{:d} green:{:d} blue:{:d},  H:{:d} S(O/L):{:d} V(P/M):{:d}, Tresh(T/F):{:d}, G goal, R robot".
                    format(red, green, blue, H, S, V, treshhold), (5, 20), cv2.FONT_HERSHEY_PLAIN, 1,
                     (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow('Camera', frame)
        if mask is not None:
            cv2.imshow('mask', mask)

        key=cv2.waitKey(1)&0xFF
        if key==ord('q'):
            ret, frame=VideoCap.read()
            status = cv2.imwrite('C:/Users/bapti/Documents/2. EPFL/MA1/Basics of Moible Robotics/MobileRoboticsGit/finalMap.png',frame)
            VideoCap.release()
            cv2.destroyAllWindows()
            break
        if key==ord('g'):
            if(HSV):
                HSVG=True
                G1, G2, G3,=H, S, V
            else:
                HSVG=False
                G1, G2, G3=blue.copy(), green.copy(), red.copy()
        if key==ord('r'):
            if(HSV):
                HSVR=True
                R1, R2, R3=H, S, V
            else:
                HSVR=False
                R1, R2, R3=blue.copy(), green.copy(), red.copy()
        if key==ord('p'):
            V=min(255, V+1)
            low=np.array([H-10, S, V])
        if key==ord('m'):
            V=max(1, V-1)
            low=np.array([H-10, S, V])
        if key==ord('o'):
            S=min(255, S+1)
            low=np.array([H-10, S, V])
        if key==ord('l'):
            S=max(1, S-1)
            low=np.array([H-10, S, V])
        if key==ord('t'):
            tresh=True
            treshhold=min(255, treshhold+1)
        if key==ord('f'):
            tresh=True
            treshhold=max(1, treshhold-1)

    return G1, G2, G3, HSVG, R1, R2, R3, HSVR, treshhold
