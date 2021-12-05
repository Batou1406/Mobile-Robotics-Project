import numpy as np

class KalmanFilterClass(object):
    def __init__(self):
        # State vector : (x, y, alpha)
        self.x=np.array([[0],
                          [0],
                          [0]])

        # input vector : (x_dot, y_dot, omega)
        self.u=np.array([[0],
                          [0],
                          [0]])

        # Matrix of system Dynamics
        self.A=np.array([[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1]])

        # Matrix of input dynamics (timestep dependent)
        self.B=np.array([[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1]])

        # Observation matrix
        self.H=np.array([[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1]])

        self.Q=np.array([[50, 0, 0],
                          [0, 50, 0],
                          [0, 0, 200]])

        self.R=np.array([[50, 0, 0],
                          [0, 50, 0],
                          [0, 0, 50]])

        self.P=np.eye(self.A.shape[1])

    def setState(self, point):
        self.x=np.array([[point[0]],
                          [point[1]],
                          [point[2]]])


    def predict(self, input, timeStep):
        self.u = np.array([[input[0]],[input[1]],[input[2]]])
        self.x=np.dot(self.A, self.x) + np.dot((timeStep*self.B), self.u)
        # Calcul de la covariance de l'erreur
        self.P=np.dot(np.dot(self.A, self.P), self.A.T)+self.Q
        return [self.x[0,0],self.x[1,0],self.x[2,0]]


    def update(self, meas):
        z = np.array([[meas[0]],[meas[1]],[meas[2]]])
        # Calcul du gain de Kalman
        S=np.dot(self.H, np.dot(self.P, self.H.T))+self.R
        K=np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        # Correction / innovation
        #self.x=np.round(self.x+np.dot(K, (z-np.dot(self.H, self.x))))
        self.x=self.x+np.dot(K, (z-np.dot(self.H, self.x)))
        I=np.eye(self.H.shape[1])
        self.P=(I-(K*self.H))*self.P

        return [self.x[0,0],self.x[1,0],self.x[2,0]]
