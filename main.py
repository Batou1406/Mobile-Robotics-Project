from VisionClass import VisionClass
from GlobalMapClass import GlobalMapClass
import ShorthestPath
from KalmanFilterClass import KalmanFilterClass
from LocalNavigator import LocalNavigator
import motionPlanning
import numpy as np
import matplotlib.pyplot as plt
import cv2
import time
from tdmclient import ClientAsync, aw


mapSize=(70,45)
globalMap=GlobalMapClass()
globalMap.setMapSize(mapSize[0],mapSize[1])
kalmanFilter=KalmanFilterClass()
vision=VisionClass(mapSize, handCalibration=True)
robot=LocalNavigator()


flag=False
vision.initialize()
time.sleep(5) #to get the of the camera done

while(not flag):
    flag=True
    vision.update()
    flag&=vision.Size()
    fig2, ax2 = plt.subplots(figsize=(20,20))
    ax2.imshow(vision.imageDraw)
    flag&=globalMap.setRobot(vision.robotDetection())
    flag&=globalMap.setGoal(vision.goalDetection())
    globalMap.setObstacles(vision.obstaclesDetection(True))
    #plt.imshow(cv2.cvtColor(vision.imageDraw,cv2.COLOR_BGR2RGB))

kalmanFilter.setState(globalMap.getRobot())

print("Astar running")
route=ShorthestPath.astar(globalMap.getObstacles(),638, 478,
                          globalMap.getRobot(), globalMap.getGoal())
globalMap.setPath(route)
print("Astar done")

if(route is not False):
    x_coords = []
    y_coords = []

    for i in (range(0,len(route))):
        x = route[i][0]
        y = route[i][1]
        x_coords.append(x)
        y_coords.append(y)

    # plot map and path
    fig, ax = plt.subplots(figsize=(20,20))
    ax = plt.gca()
    ax.invert_yaxis()
    ax.imshow(vision.obstaclesDetection(False), cmap=plt.cm.Dark2)
    ax.imshow(globalMap.getObstacles(), cmap=plt.cm.Dark2, alpha=0.3)
    ax.scatter(globalMap.getRobot()[0],globalMap.getRobot()[1], marker = "*", color = "yellow", s = 200)
    ax.scatter(globalMap.getGoal()[0],globalMap.getGoal()[1], marker = "*", color = "red", s = 200)
    ax.plot(x_coords,y_coords, color = "black")
    plt.show()


cv2.startWindowThread()
cv2.imshow('Robot', vision.imageDraw)
input=[0,0,0]
while(True):
    robotPos=kalmanFilter.predict(input,0.1)
    vision.update()
    meas = vision.robotDetection()
    if meas is not False:
        robotPos=kalmanFilter.update(meas)

    globalMap.setRobot(robotPos)
    motorSpeed, omega = aw(robot.run(motionPlanning.getMotionAngle(globalMap.getPath(),globalMap.getRobot())))
    #input=[motorSpeed*np.cos(globalMap.getRobot()[2]),motorSpeed*np.sin(globalMap.getRobot()[2]), omega]
    image = vision.imageDraw
    cv2.circle(image, (int(globalMap.getRobot()[0]), int(globalMap.getRobot()[1])), 25, (255, 0, 0), 2)
    cv2.arrowedLine(image,(int(globalMap.getRobot()[0]), int(globalMap.getRobot()[1])),(int(globalMap.getRobot()[0]+30*np.cos(globalMap.getRobot()[2])),
                    int(globalMap.getRobot()[1]+30*np.sin(globalMap.getRobot()[2]))),color=(255, 0, 0),thickness=4, tipLength=0.2)
    for i in range(len(x_coords)):
        cv2.circle(image, (x_coords[i], y_coords[i]), 1, (0, 0, 255), 2)

    cv2.putText(image, "angle{:d}, robot : x {:d}, y {:d}".format(int(motionPlanning.getMotionAngle(globalMap.getPath(),globalMap.getRobot())),int(globalMap.getRobot()[0]),int(globalMap.getRobot()[1])),(5, 20), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imshow('Robot', image)
    cv2.waitKey(1)
