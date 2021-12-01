import cv2
import numpy as np
import calibrate

class VisionClass(object):
    def __init__(self,mapSize):
        self.ratioX=0
        self.ratioY=0
        self.cornerX=0
        self.cornerY=0
        self.gridX=mapSize[0]
        self.gridY=mapSize[1]
        self.image=None
        self.imageDraw=None
        self.VideoCap=None
        self.lowGoal=np.zeros(3)
        self.highGoal=np.zeros(3)
        self.lowRobot=np.zeros(3)
        self.highRoobot=np.zeros(3)
        self.map=None
        self.mask=None
        self.erode=None
        self.binary=None
        self.gray=None

    def initialize(self):
        blueG, greenG, redG, blueR, greenR, redR=calibrate.calibration()
        print(blueG, greenG, redG, blueR, greenR, redR)
        self.lowGoal=np.array([blueG-40,greenG-40,redG-40])
        self.highGoal=np.array([blueG+40,greenG+40,redG+40])
        self.lowRobot=np.array([blueR-25,greenR-25,redR-25])
        self.highRobot=np.array([blueR+25,greenR+25,redR+25])
        self.VideoCap=cv2.VideoCapture(0)
        ret, frame=self.VideoCap.read()
        self.image=frame


    def Size(self):
        #filter
        gray=cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        self.map=gray
        blur=cv2.GaussianBlur(gray, (7, 7), 0.5)
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

    def update(self):
        ret, frame=self.VideoCap.read()
        self.image=frame

    def pixelToCM(self, x_pixel, y_pixel):
        x_cm=round(((x_pixel-self.cornerX)*self.gridX)/self.ratioX)
        y_cm=round(((y_pixel-self.cornerY)*self.gridY)/self.ratioY)
        y_cm=self.gridY-y_cm-1
        return x_cm, y_cm


    def goalDetection(self):
        mask=cv2.inRange(self.image, self.lowGoal, self.highGoal)
        self.mask=mask
        map_mask_goal=cv2.bitwise_and(self.image, self.image, mask=mask)
        self.erode=map_mask_goal
        gray_goal=cv2.cvtColor(map_mask_goal, cv2.COLOR_BGR2GRAY)
        self.binary=gray_goal
        circles=cv2.HoughCircles(gray_goal, cv2.HOUGH_GRADIENT,
                                   1, 20, param1=100, param2=10,
                                   minRadius=5, maxRadius=500)
        if circles is None:
            print("Warning: No goal found, take another picture.")
        else:
            cv2.circle(self.imageDraw, (int(circles[0, 0, 0]), int(circles[0, 0, 1])), 2, (0, 255, 0), 5)
            goalX, goalY=self.pixelToCM(circles[0, 0, 0],circles[0, 0, 1]) #goal position in x,y form
            return [goalX, goalY]


    def robotDetection(self):
        # Apply mask to image to get the goal (red circle)
        mask = cv2.inRange(self.image, self.lowRobot, self.highRobot)
        self.mask=mask
        map_mask_thymio = cv2.bitwise_and(self.image, self.image, mask=mask)
        self.erode=map_mask_thymio
        # Convert image to gray
        gray_thymio = cv2.cvtColor(map_mask_thymio, cv2.COLOR_BGR2GRAY)
        self.binary=gray_thymio
        # Detect circles
        circles = cv2.HoughCircles(gray_thymio, cv2.HOUGH_GRADIENT,
                                   1, 20, param1=100, param2=10,
                                   minRadius=0, maxRadius=100)

        # Output
        if circles.shape[1]<2:
            print("Warning: No thymio found, take another picture.")
        else:
            if circles[0][0][2] > circles[0][1][2]:
                xp,yp,rp = circles[0][1]
                xg,yg,rg = circles[0][0]
            else:
                xp,yp,rp = circles[0][0]
                xg,yg,rg = circles[0][1]

            direction = [xp-xg,yp-yg]

            thymio_x = xg
            thymio_y = yg
            thymio_theta = -np.arctan2(direction[1], direction[0])
            cv2.circle(self.imageDraw, (int(thymio_x), int(thymio_y)), 2, (0, 255, 0), 5)
            thymio_x_cm, thymio_y_cm = self.pixelToCM(thymio_x, thymio_y)
            return [thymio_x_cm, thymio_y_cm, thymio_theta]


    def preprocessing(self):
        # Convert to RGB
        map_rgb = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)

        # Blur Image
        map_blur = cv2.blur(map_rgb,(10,10))

        # Convert to RGB
        map_rgb = cv2.cvtColor(map_blur, cv2.COLOR_BGR2RGB)

        # Apply Bilateral filter
        map_bilateral = cv2.bilateralFilter(map_rgb,5,15,15)

        # Create Binary Image
        map_gray = cv2.cvtColor(map_bilateral, cv2.COLOR_RGB2GRAY)
        _, map_binary = cv2.threshold(map_gray, 70, 255, cv2.THRESH_BINARY_INV)

        # Opening to remove noise
        kernel_morph = np.ones((10,10),np.uint8)
        map_clean = cv2.morphologyEx(map_binary, cv2.MORPH_OPEN, kernel_morph)

        kernel_erode = np.ones((60,60),np.uint8)
        kernel_dilate = np.ones((140,140),np.uint8)
        map_obstacle = cv2.erode(map_clean, kernel_erode, iterations=1)
        map_occupancy = cv2.dilate(map_obstacle, kernel_dilate, iterations=1)

        return map_occupancy


    def obstaclesDetection(self):

        map_occupancy=self.preprocessing()

        width=self.gridX
        height=self.gridY # Size of the grid

        occupancy_grid = np.zeros((height,width))
        nx=round(self.ratioX/width)
        ny=round(self.ratioY/height)

        for i in range(height):
            for j in range(width):
                result = np.sum(map_occupancy[(nx*i+self.cornerX):(nx*(i+1)+self.cornerX),
                                              (ny*j+self.cornerY):(ny*(j+1)+self.cornerY)])
                if result > 0 :
                    occupancy_grid[height-i-1,j] = 1

        return occupancy_grid
