import numpy as np

class GlobalMapClass(object):
    def __init__(self):
        #robot position as a vector (x,y,alpha),
        #(x,y) as float number. Integer is the cell, floating point is the position inside the cell
        # alpha between [0,2pi]
        self.robot=[0,0,0]

        # goal postion as a vector (x,y)
        self.goal=[0,0]

        #obstacles grid with zero and one of size mapSize
        self.obstaclesGrid=None

        #empty list with [[x,y],[x,y],[x,y],...] position as path
        self.path=None

        # vector with map size [[height],[width]]
        self.mapSize=[0,0]

    def setRobot(self, robotPos):
        if(robotPos is not False):
            #robotPos as [x,y,aplha] coordinate
            if(robotPos[2]>8):#ie, pos is available but not the angle (outside 2pi)
                self.robot=[robotPos[0], robotPos[1], self.robot[2]]
            else:
                self.robot=robotPos
            return True
        return False

    def setGoal(self, goalPos):
        if(goalPos is not False):
            #goalPos as [x,y] coordinate
            self.goal=goalPos
            return True
        return False

    def setPath(self, pathList):
        #pathList contains a list of (x,y) coordinates
        self.path=pathList

    def setObstacles(self, obstacles):
        #pathObstacles contains a list of (x,y) coordinates
        self.obstaclesGrid=obstacles

    def setMapSize(self, height, width):
        self.mapSize=[height,width]

    def getPath(self):
        return self.path

    def getObstacles(self):
        return self.obstaclesGrid

    def getMapSize(self):
        return self.mapSize

    def getRobot(self):
        return self.robot

    def getGoal(self):
        return self.goal
