import numpy as np
from scipy.interpolate import interp1d

sensor_measurements = np.array([i for i in range(0,21)])
sensor_distances = np.array([5120, 4996, 4964, 4935, 4554, 4018, 3624, 3292, 2987, 
        2800, 2580, 2307, 2039, 1575, 1127, 833, 512, 358, 157, 52, 0])

def sensor_val_to_cm_dist(val):
    """
    Returns the distance corresponding to the sensor value based 
    on the sensor characteristics
    :param val: the sensor value that you want to convert to a distance
    :return: corresponding distance in cm
    """
    if val == 0:
        return np.inf
    
    f = interp1d(sensor_measurements, sensor_distances)
    return f(val).item()