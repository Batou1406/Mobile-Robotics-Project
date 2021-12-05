import cv2
import numpy as np
import calibrate

class VisionClass(object):
    def __init__(self,mapSize, handCalibration):
        self.ratioX=0
        self.ratioY=0
        self.cornerX=0
        self.cornerY=0
        self.gridX=mapSize[0]
        self.gridY=mapSize[1]
        self.image=None
        self.imageDraw=None
        self.VideoCap=None
        self.lowGoal=[84, 38, 137]
        self.highGoal=[124, 78, 177]
        self.lowRobot=[83, 75, 75]
        self.highRoobot=[103,255,255]
        self.HSWG=False
        self.HSWR=True
        self.handCalibration=handCalibration
        self.tresh=70
        self.map=None
        self.mask=None
        self.erode=None
        self.binary=None
        self.gray=None

    def initialize(self):
        if(self.handCalibration):
            G1, G2, G3, HSVG, R1, R2, R3, HSVR, tresh=calibrate.calibration()
            self.tresh=tresh
            self.HSVG=HSVG
            self.HSVR=HSVR
            if(HSVG):
                self.lowGoal=np.array([G1-10,G2,G3])
                self.highGoal=np.array([G1+10,255,255])
            else:
                self.lowGoal=np.array([G1-30,G2-30,G3-30])
                self.highGoal=np.array([G1+30,G2+30,G3+30])
            if(HSVR):
                self.lowRobot=np.array([R1-15,R2,R3])
                self.highRobot=np.array([R1+15,255,255])
            else:
                self.lowRobot=np.array([R1-25,R2-25,R3-25])
                self.highRobot=np.array([R1+25,R2+25,R3+25])

        self.VideoCap=cv2.VideoCapture(0)
        ret, frame=self.VideoCap.read()
        self.image=frame


    def Size(self):
        #filter
        gray=cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        self.map=gray
        blur=cv2.GaussianBlur(gray, (3, 3), 0.5)
        self.mask=blur
        edge=cv2.Canny(blur, 0, 50, 3)
        self.erode=edge

        #find Countours
        contours, hierarchy=cv2.findContours(edge, cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
        #if no contours are found
        if(len(contours) < 1):
            return False

        #take largest rectangle
        x,y,w,h = 0,0,0,0
        for cnt in contours:
            x_new,y_new,w_new,h_new=cv2.boundingRect(cnt)
            if(w_new*h_new>w*h):
                x,y,w,h=x_new,y_new,w_new,h_new

        #dray rectangle for display purpose
        self.imageDraw=self.image.copy()
        cv2.rectangle(self.imageDraw, (x,y), pt2=(x+w,y+h), color=(255,0,0), thickness=10)

        #set up variables for grid conversion
        self.cornerX=x
        self.cornerY=y
        self.ratioX=w
        self.ratioY=h
        return True


    def finish(self):
        self.VideoCap.release()
        cv2.destroyAllWindows()

    def update(self):
        ret, frame=self.VideoCap.read()
        self.image=frame
        self.imageDraw=frame

    def display(self):
        cv2.imshow('Image Draw', self.imageDraw)

    def pixelToCM(self, x_pixel, y_pixel):
        x_cm=round(((x_pixel-self.cornerX)*self.gridX)/self.ratioX)
        y_cm=round(((y_pixel-self.cornerY)*self.gridY)/self.ratioY)
        return x_cm, y_cm


    def goalDetection(self):
        filter=cv2.blur(self.image, (3, 3))
        if(self.HSVG):
            # Convert image to HSV
            map_hsv=cv2.cvtColor(filter, cv2.COLOR_BGR2HSV)
            mask=cv2.inRange(map_hsv, self.lowGoal, self.highGoal)
        else:
            mask=cv2.inRange(filter, self.lowGoal, self.highGoal)

        points=[]
        elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        elements=sorted(elements, key=lambda x:cv2.contourArea(x), reverse=True)
        surface = 15
        for element in elements:
            if cv2.contourArea(element)>surface:
                ((x, y), rayon)=cv2.minEnclosingCircle(element)
                points.append(np.array([int(x), int(y)]))
        if (len(points)>0):
            cv2.circle(self.imageDraw, (points[0][0], points[0][1]), 10, (0, 0, 255), 2)
            goalX, goalY=self.pixelToCM(points[0][0], points[0][1]) #goal position in x,y form
            return [goalX, goalY]
        else:
            print("Warning: No goal found, take another picture.")
            return False


    def robotDetection(self):
        filter=cv2.blur(self.image, (3, 3))
        if(self.HSVR):
            # Convert image to HSV
            map_hsv = cv2.cvtColor(filter, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(map_hsv, self.lowRobot, self.highRobot)
        else:
            mask=cv2.inRange(filter, self.lowRobot, self.highRobot)

        points=[]
        elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        elements=sorted(elements, key=lambda x:cv2.contourArea(x), reverse=True)
        surface = 15
        for element in elements:
            if cv2.contourArea(element)>surface:
                ((x, y), rayon)=cv2.minEnclosingCircle(element)
                points.append(np.array([int(x), int(y)]))

        if (len(points)>0):
            cv2.circle(self.imageDraw, (points[0][0], points[0][1]), 15, (0, 255, 0), 2)
            robotX,robotY=self.pixelToCM(points[0][0], points[0][0])
            theta=10
            if(len(points)>1):
                direction=[points[0][0]-points[1][0],points[0][1]-points[1][1]]
                theta = np.arctan2(direction[1], direction[0])
                cv2.arrowedLine(self.imageDraw,(points[0][0], points[0][1]),(int(points[0][0]+30*np.cos(theta)),
                                int(points[0][1]+30*np.sin(theta))),color=(0, 255, 0),thickness=3, tipLength=0.2)
            thymio_x_cm, thymio_y_cm = self.pixelToCM(points[0][0], points[0][1])
            return [thymio_x_cm, thymio_y_cm, theta]
        else:
            print("Warning: No Robot found, take another picture.")
            return False


    def preprocessing(self,expend):
        # Convert to RGB
        map_rgb = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)

        # Blur Image
        map_blur = cv2.blur(map_rgb,(10,10))

        # Apply Bilateral filter
        map_bilateral = cv2.bilateralFilter(map_rgb,5,15,15)

        # Create Binary Image
        map_gray = cv2.cvtColor(map_bilateral, cv2.COLOR_RGB2GRAY)
        _, map_binary = cv2.threshold(map_gray, self.tresh, 255, cv2.THRESH_BINARY_INV)

        # Opening to remove noise
        kernel_morph = np.ones((10,10),np.uint8)
        map_clean = cv2.morphologyEx(map_binary, cv2.MORPH_OPEN, kernel_morph)

        kernel_erode = np.ones((10,10),np.uint8) #60,60
        kernel_dilate = np.ones((30,30),np.uint8) #140,140
        map_occupancy = cv2.erode(map_clean, kernel_erode, iterations=1)
        if(expend):
            map_occupancy = cv2.dilate(map_occupancy, kernel_dilate, iterations=1)

        return map_occupancy


    def obstaclesDetection(self, expend):
        map_occupancy=self.preprocessing(expend)

        width=self.gridX
        height=self.gridY # Size of the grid

        occupancy_grid = np.zeros((height,width))
        nx=self.ratioX/width
        ny=self.ratioY/height

        for i in range(height):
            for j in range(width):
                result = np.sum(map_occupancy[round(nx*i+self.cornerX):round(nx*(i+1)+self.cornerX),
                                              round(ny*j+self.cornerY):round(ny*(j+1)+self.cornerY)])
                if result > 0 :
                    occupancy_grid[i,j] = 1

        return occupancy_grid
