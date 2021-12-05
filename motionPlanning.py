import numpy as np

def getMotionAngle(path, robot):
    previous=abs(path[0][0]-robot[0]) + abs(path[0][1]-robot[1])
    index=0
    for i in length(path)-1:
         if abs(path[i][0]-robot[0]) + abs(path[i][1]-robot[1]) <= previous:
            previous=abs(path[i][0]-robot[0]) + abs(path[i][1]-robot[1])
            index=i
    direction=[path[index+1][0]-points[index][0],points[index+1][1]-points[index][1]]
    angleToGoal=np.arctan2(direction[1], direction[0])
    return angleToGoal-robot[3]
