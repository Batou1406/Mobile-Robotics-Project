import numpy as np

class KalmanFilter(object):
    def __init__(self, dt, point):
        self.dt=dt

        # State vector
        self.E=np.matrix([[point[0]], [point[1]], [0], [0]])

        # Dynamics matrix
        self.A=np.matrix([[1, 0, self.dt, 0],
                          [0, 1, 0, self.dt],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

        # Matrice d'observation, on observe que x et y
        self.H=np.matrix([[1, 0, 0, 0],
                          [0, 1, 0, 0]])

        self.Himage=np.matrix([[1, 0, 0, 0],
                              [0, 1, 0, 0]])

        self.Hwheels=np.matrix([[0, 0, 1, 0],
                                 [0, 0, 0, 1]])

        self.Hfull=np.matrix([[1, 0, 0, 0],
                              [0, 1, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

        self.Q=np.matrix([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

        self.R=np.matrix([[1, 0],
                          [0, 1]])

        self.Rimage=np.matrix([[1, 0],
                               [0, 1]])


        self.Rwheels=np.matrix([[5, 0],
                               [0, 5]])

        self.Rfull=np.matrix([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [0, 0, 5, 0],
                               [0, 0, 0, 5]])

        self.P=np.eye(self.A.shape[1])  #corrigé en fct des erreurs

    def update2(self, z):
        # Calcul du gain de Kalman
        S=np.dot(self.H, np.dot(self.P, self.H.T))+self.R
        K=np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        # Correction / innovation
        self.E=np.round(self.E+np.dot(K, (z-np.dot(self.H, self.E))))
        I=np.eye(self.H.shape[1])
        self.P=(I-(K*self.H))*self.P

        return self.E

    def predict(self):
        self.E=np.dot(self.A, self.E) + B*u
        # Calcul de la covariance de l'erreur
        self.P=np.dot(np.dot(self.A, self.P), self.A.T)+self.Q
        return self.E

    def updateImage(self, z):
        # Kalman gain
        S=np.dot(self.Himage, np.dot(self.P, self.Himage.T))+self.Rimage
        K=np.dot(np.dot(self.P, self.Himage.T), np.linalg.inv(S))

        # Correction / innovation
        self.E=np.round(self.E+np.dot(K, (z-np.dot(self.Himage, self.E))))
        I=np.eye(self.Himage.shape[1])
        self.P=(I-(K*self.Himage))*self.P

        return self.E

    def updateWheels(self, z):
        #Kalman gain
        S=np.dot(self.Hwheels, np.dot(self.P, self.Hwheels.T))+self.Rwheels
        K=np.dot(np.dot(self.P, self.Hwheels.T), np.linalg.inv(S))
        #correction / innovation
        self.E=np.round(self.E+np.dot(K, (z-np.dot(self.Hwheels, self.E))))
        I=np.eye(self.Hwheels.shape[1])
        self.P=(I-(K*self.Hwheels))*self.P

        return self.E

    def update(self, z, meas):

        if(meas == 'full'):
            H = self.Hfull
            R = self.Rfull
        elif(meas == 'speed'):
            H = self.Hwheels
            R = self.Rwheels
        elif(meas == 'image'):
            H = self.Himage
            R = self.Rimage

        # Kalman gain
        S=np.dot(H, np.dot(self.P, H.T))+R
        K=np.dot(np.dot(self.P, H.T), np.linalg.inv(S))

        # Correction / innovation
        self.E=np.round(self.E+np.dot(K, (z-np.dot(H, self.E))))
        I=np.eye(H.shape[1])
        self.P=(I-(K*H))*self.P

        return self.E
