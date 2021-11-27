class GlobalMapClass(object):
    def __init__(self):
        #robot position as a vector (x,y,alpha),
        #(x,y) as float number. Integer is the cell, floating point is the position inside the cell
        # alpha between [0,2pi]
        self.robot=np.matrix([[0],
                              [0],
                              [0]])

        # goal postion as a vector (x,y)
        self.goal=np.matrix([[0],
                             [0]])

        #empty list with [[x,y],[x,y],[x,y],...] obstacles position
        self.obstacles=[]

        #empty list with [[x,y],[x,y],[x,y],...] position as path
        self.path=[]

        # vector with map size [[height],[width]]
        self.mapSize=np.matrix([[0],
                                [0]])

    def setPos(self, robotPos):
        #robotPos as [[x],[y],[alpha]]
        self.robot=robotPos

    def setGoal(self, goalPos):
        #goalPos as [[x],[y]] coordinate
        self.goal=goalPos

    def setPath(self, pathList):
        #pathList contains a list of (x,y) coordinates
        self.path=pathList

    def setObstacles(self, obstaclesList):
        #pathObstacles contains a list of (x,y) coordinates
        self.obstacles=obstaclesList

    def setMapSize(self, height, width):
        self.mapSize=np.matrix([[height],
                                  [width]])

    def getPath(self, i):
        return self.path[i]

    def getObstacles(self):
        return self.obstacles

    def getMapSize(self):
        return self.mapSize

    def getPos(self):
        return self.robot

    def getGoal(self):
        return self.goal
