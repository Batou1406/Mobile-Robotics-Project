import numpy as np

def getMotionAngle2(path, robot):
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


def getMotionAngle(path, robot):
    # find closest point
    min_dist = abs(path[0][0]-robot[0]) + abs(path[0][1]-robot[1])
    index=0
    for i in range(10,len(path)):
        dist=abs(path[i][0]-robot[0]) + abs(path[i][1]-robot[1])
        if (dist<min_dist):
            min_dist=dist
            index=i

    # determine if robot is left or right of path
    path_vect=[path[index-10][0]-path[index][0],path[index-10][1]-path[index][1]]
    robot_vect=[path[index][0]-robot[0],path[index][1]-robot[1]]
    angle = np.arccos(np.dot(path_vect, robot_vect))

    # choose correction direction
    if (angle>180):
        #error_terms.append(dist)
        error_terms=min_dist
    else:
        #error_terms.append(-dist)
        error_terms=-min_dist

    # PID gains (to tune)
    Kp = 10
    Ki = 0.1
    Kd = 1

    # get angle correction value (PID controller)
    #angle_correction=Kp*error_terms[-1]+Ki*sum(error_terms)+Kd*(error_terms[-1]-error_terms[-2])
    angle_correction=Kp*error_terms

    return angle_correction, error_terms
