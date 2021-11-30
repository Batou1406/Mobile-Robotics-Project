import cv2
import numpy as np

class VisionClass(object):
    def __init__(self,mapSize):
        self.ratioX=0
        self.ratioY=0
        self.cornerX=0
        self.cornerY=0
        self.gridX=mapSize[0]
        self.gridY=mapSize[1]
        self.image=None
        self.VideoCap=None
        self.map=None
        self.mask=None
        self.erode=None
        self.binary=None
        self.gray=None

    def initialize(self):
        #self.VideoCap=cv2.VideoCapture(0)
        #ret, frame=self.VideoCap.read()
        #self.image=frame

        self.image = cv2.imread('Final_map.jpg')

        #BGR to HSV
        map_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        self.map = map_hsv

        # range of color for contours
        #low_blue = np.array([94, 80, 2])
        #high_blue = np.array([126, 255, 255])
        low_blue = np.array([90, 80, 10])
        high_blue = np.array([130, 255, 255])

        # mask to detect the color
        mask = cv2.inRange(map_hsv, low_blue, high_blue)
        map_mask_countour = cv2.bitwise_and(self.image, self.image, mask=mask)

        self.mask = map_mask_countour

        # filtering
        kernel_remove = np.ones((10,10),np.uint8)
        map_erode = cv2.erode(map_mask_countour, kernel_remove, iterations=1)

        self.erode = map_erode

        # convert to gray
        gray = cv2.cvtColor(map_erode, cv2.COLOR_RGB2GRAY)

        self.gray = gray

        # convert to binary
        _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)

        self.binary = binary

        contours,hierarchy=cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        if(len(contours) < 1):
            return False
        x,y,w,h=cv2.boundingRect(contours[-1])
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
        # Values for detecting the color red
        low_red=np.array([150, 100, 50])
        high_red=np.array([200, 255, 255])

        # Convert image to HSV
        map_hsv=cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        # Apply mask to image to get the goal (red circle)
        mask=cv2.inRange(map_hsv, low_red, high_red)
        map_mask_goal=cv2.bitwise_and(self.image, self.image, mask=mask)

        # Convert image to gray
        gray_goal=cv2.cvtColor(map_mask_goal, cv2.COLOR_BGR2GRAY)

        # Detect circles
        circles=cv2.HoughCircles(gray_goal, cv2.HOUGH_GRADIENT,
                                   1, 20, param1=100, param2=10,
                                   minRadius=20, maxRadius=100)

        # Output
        if circles is None:
            print("Warning: No goal found, take another picture.")
            find_goal(self.image) # Remove argument when code is modified
        else:
            goalX, goalY=self.pixelToCM(circles[0, 0, 0],circles[0, 0, 1]) #goal position in x,y form
            return [goalX, goalY]


    def robotDetection(self):
        # Values for detecting the color green
        low_green = np.array([10, 100, 50])
        high_green = np.array([130, 255, 255])

        # Convert image to HSV
        map_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        # Apply mask to image to get the goal (red circle)
        mask = cv2.inRange(map_hsv, low_green, high_green)
        map_mask_thymio = cv2.bitwise_and(self.image, self.image, mask=mask)

        # Convert image to gray
        gray_thymio = cv2.cvtColor(map_mask_thymio, cv2.COLOR_BGR2GRAY)

        # Detect circles
        circles = cv2.HoughCircles(gray_thymio, cv2.HOUGH_GRADIENT,
                                   1, 20, param1=100, param2=10,
                                   minRadius=0, maxRadius=100)

        # Output
        if circles.shape[1]<2:
            print("Warning: No thymio found, take another picture.")
            find_thymio(self.image) # Remove argument when code is modified
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
            thymio_theta = np.arctan2(direction[1], direction[0])
            thymio_x_cm, thymio_y_cm = self.pixelToCM(thymio_x, thymio_y)
            return [thymio_x_cm, thymio_y_cm, -thymio_theta]


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
