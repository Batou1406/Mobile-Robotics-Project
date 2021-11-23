import numpy as np

class KalmanFilter(object):
    def __init__(self, point):
        # State vector : (x, y, alpha)
        self.x=np.matrix([[point[0]], [point[1]],[point[2]]])

        # input vector : (x_dot, y_dot, omega)
        self.u=np.matrix([[0], [0], [0]])

        # Matrix of system Dynamics
        self.A=np.matrix([[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1]])

        # Matrix of input dynamics (timestep dependent)
        self.B=np.matrix([[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1]])

        # Observation matrix
        self.H=np.matrix([[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1]])

        self.Q=np.matrix([[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1]])

        self.R=np.matrix([[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1]])

        self.P=np.eye(self.A.shape[1])

    def predict(self, input, timeStep):
        self.u = input
        self.x=np.dot(self.A, self.x) + np.dot((timeStep*self.B), self.u)
        # Calcul de la covariance de l'erreur
        self.P=np.dot(np.dot(self.A, self.P), self.A.T)+self.Q
        return self.x

    def update(self, z):
        # Calcul du gain de Kalman
        S=np.dot(self.H, np.dot(self.P, self.H.T))+self.R
        K=np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        # Correction / innovation
        self.x=np.round(self.x+np.dot(K, (z-np.dot(self.H, self.x))))
        I=np.eye(self.H.shape[1])
        self.P=(I-(K*self.H))*self.P

        return self.x
