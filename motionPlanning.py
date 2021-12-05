import numpy as np

def getMotionAngle(path, robot):
    previous=abs(path[0][0]-robot[0]) + abs(path[0][1]-robot[1])
    index=0
    for i in range(10,len(path)):
         if abs(path[i][0]-robot[0]) + abs(path[i][1]-robot[1]) <= previous:
            previous=abs(path[i][0]-robot[0]) + abs(path[i][1]-robot[1])
            index=i
    direction=[path[index-10][0]-path[index][0],path[index-10][1]-path[index][1]]
    angleToGoal=np.arctan2(direction[1], direction[0])%(2*np.pi)
    if((180*(angleToGoal-robot[2]))/np.pi < 180):
        return (180*(angleToGoal-robot[2]))/np.pi
    else:
        return ((180*(angleToGoal-robot[2]))/np.pi-360)
