#main file for making the project work
"""
initialisation
"""
#librairy
import VisionClass
import GlobalMapClass
import ShorthestPath as SP
import KalmanFilterClass

#flag
kidnap=False
bloqued=False

#variables declaration
globalMap=GlobalMapClass()
vision=visionClass()
kalmanFilter=KalmanFilterClass()

#map initialisation
vision.begin()
gloablMap.mapSize(vision.height, vision.width)
globalMap.setPos(vision.robotDetection())
globalMap.setGoal(vision.goalDetection())
globalMap.setObstacles(vision.obstaclesDetection())
gloablMap.setPath(SP.aStar(globalMap.getMapSize(), globalMap.getObstacles()))
