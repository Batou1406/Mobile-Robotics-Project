#main file for making the project work
"""
initialisation
"""
#librairy
from VisionClass import VisionClass
from GlobalMapClass import GlobalMapClass
from ShorthestPath import ShorthestPath as SP
from KalmanFilterClass import KalmanFilterClass
import MotionClass

#flag
kidnap=False
bloqued=False
reachedGoal=False

#variables declaration
globalMap=GlobalMapClass()
vision=visionClass()
kalmanFilter=KalmanFilterClass()
input=np.matrix([[0],[0],[0]])
timeStep=0
measurement=None
angleToGoal=None

#Vision initialisation
vision.initialize()

#Kalman filter initialisation
kalmaFilter.setState(vision.robotDetection())

#map initialisation
gloablMap.mapSize(vision.height, vision.width)
globalMap.setPos(kalmanFilter.x)
globalMap.setGoal(vision.goalDetection())
globalMap.setObstacles(vision.obstaclesDetection())

#Shorthest path computation
gloablMap.setPath(SP.aStar(globalMap.getMapSize(), globalMap.getObstacles()))



"""
Main Loop
"""
while(not(reachedGoal)):
    # get robot position
    kalmanFilter.predict(input, timeStep)
    measurement=vision.robotDetection()
    if(measurement):
        kalmanFilter.update(measurement)
    globalMap.setPos(kalmanFilter.x)

    #compute and apply input


    # re-initialise if bloqued or kidnapp
    if(bloqued or kidnap):
        vision.initialize()
        gloablMap.mapSize(vision.height, vision.width)
        globalMap.setPos(vision.robotDetection())
        globalMap.setGoal(vision.goalDetection())
        globalMap.setObstacles(vision.obstaclesDetection())
        gloablMap.setPath(SP.aStar(globalMap.getMapSize(), globalMap.getObstacles()))

    #check if goal is reached
    if(round(globalMap.robot) == round(gloablMap.goal)):
        reachedGoal=True