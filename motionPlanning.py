import numpy as np

def getMotionAngle(path, robot):
    previous=abs(path[0][0]-robot[0]) + abs(path[0][1]-robot[1])
    index=0
    for i in range(len(path)-1):
         if abs(path[i][0]-robot[0]) + abs(path[i][1]-robot[1]) <= previous:
            previous=abs(path[i][0]-robot[0]) + abs(path[i][1]-robot[1])
            index=i
    direction=[path[index][0]-path[index+1][0],path[index][1]-path[index+1][1]]
    angleToGoal=np.arctan2(direction[1], direction[0])
    #print((-(angleToGoal-robot[2]))%(2*np.pi))
    #return (-(angleToGoal-robot[2]))%(2*np.pi)
    if((180*(angleToGoal-robot[2]))/np.pi < 180):
        return (180*(angleToGoal-robot[2]))/np.pi
    else:
        return ((180*(angleToGoal-robot[2]))/np.pi-360)
