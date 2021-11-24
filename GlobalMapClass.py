class GlobalMapClass(object):
    def __init__(self):
        #robot position as a vector (x,y,alpha)
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

    def setPos(robotPos):
        #robotPos as [[x],[y],[alpha]]

    def setGoal(goalPos):
        #goalPos as [[x],[y]] coordinate
        self.goal = goalPos

    def setPath(self, pathList):
        #pathList contains a list of (x,y) coordinates
        self.path = pathList

    def setObstacles(self, obstaclesList):
        #pathObstacles contains a list of (x,y) coordinates
        self.obstacles = obstaclesList

    def getPath(i):
        return self.path[i]
