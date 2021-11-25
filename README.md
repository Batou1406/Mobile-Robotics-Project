# Project of Mobile Robotics descrpition
structure of the project : the strcuture can be found under the machine state diagram
the project in running in the main file. It calls all the requires functions, classes and variables from other files.

## Function, classes and variables.
### kalmanFilterClass
file with the kalman filter class in it with :
#### variables :
x : state vector (x,y,alpha)
u : input vector (x_dot, y_dot, alpha_dot)
#### functions :
predict(input, timeStep) : return predicted next state
update(z) : return corrected next state with measurement z

### GloabalMapClass
file with the global map in it :
#### variables :
robot : robot position (x,y,alpha)
goal : goal position (x,y)
obstacles : list of obstacles
path : list of position for the shortest path
mapSize : vector with (height, width) of the map
##### functions :
setPos(robotPos) :
setGoal(goalPos) :
setPath(pathList) :
setObstacles(obstaclesList) :
setMapSize(height, width) :
getPath(i) :
getObstacles() :
getMapSize() :

### visionClass
file with all the vision functions:
#### variables :
vision.height :
vision.width :
#### functions :
vision.begin() :
vision.robotDetection() :
vision.goalDetection() :
vision.obstaclesDetection() :

### ShorthestPath
file the shortest path implementation :
#### Variables :
#### fucntions :
aStar(mapSize, obstaclesList) :
