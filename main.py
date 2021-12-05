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

route=ShorthestPath.astar(globalMap.getObstacles(),globalMap.getMapSize()[0], globalMap.getMapSize()[1],
                          globalMap.getRobot(), globalMap.getGoal())
globalMap.setPath(route)


fig, ax = plt.subplots(figsize=(20,20))
ax = plt.gca()
ax.invert_yaxis()
ax.imshow(vision.obstaclesDetection(False), cmap=plt.cm.Dark2)
ax.imshow(globalMap.getObstacles(), cmap=plt.cm.Dark2, alpha=0.3)
ax.scatter(globalMap.getRobot()[0],globalMap.getRobot()[1], marker = "*", color = "yellow", s = 200)
ax.scatter(globalMap.getGoal()[0],globalMap.getGoal()[1], marker = "*", color = "red", s = 200)
#ax.plot(x_coords,y_coords, color = "black")
plt.show()

time.sleep(15)

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

while(True):
    vision.update()
    globalMap.setRobot(vision.robotDetection())
    aw(robot.run(motionPlanning.getMotionAngle(globalMap.getPath(),globalMap.getRobot())))
    image = vision.imageDraw
    for i in range(len(x_coords)):
        cv2.circle(image, (int(x_coords[i]*vision.ratioX/vision.gridX + vision.cornerX), int(y_coords[i]*vision.ratioY/vision.gridY + vision.cornerY)), 1, (0, 0, 255), 2)

    cv2.putText(image, "angle{:d}".format(int(motionPlanning.getMotionAngle(globalMap.getPath(),globalMap.getRobot()))),(5, 20), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imshow('Robot', image)
    cv2.waitKey(1)
