from tdmclient import ClientAsync, aw
import numpy as np
from scipy.interpolate import interp1d
import math

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

class LocalNavigator:
    def __init__(self):
        self.client = ClientAsync()
        self.node = aw(self.client.wait_for_node())

        aw(self.node.lock())
        aw(self.node.wait_for_variables())

        self.dist_threshold = 1800
        self.motor_speed = 300
        self.sensor_vals = list(self.node['prox.horizontal'])
        self.verbose = True # whether to print status message or not
        self.direction = self.val_to_angle(90)

    def val_to_angle(self, val):
        return val * math.pi / 180

    def motor(self, l_speed=500, r_speed=500):
        if self.verbose:
            print("\tSetting speed: ", l_speed, r_speed)
        return {
                "motor.left.target": [l_speed],
                "motor.right.target": [r_speed],
                }
    
    async def turn_left(self):
        await self.node.set_variables(self.motor(-self.motor_speed, self.motor_speed))
    
    async def turn_right(self):
        await self.node.set_variables(self.motor(self.motor_speed, -self.motor_speed))

    async def forward(self):
        await self.node.set_variables(self.motor(self.motor_speed, self.motor_speed))

    async def backward(self):
        await self.node.set_variables(self.motor(-self.motor_speed, -self.motor_speed))

    async def avoid(self, angle):
        if self.verbose:
            print("\tSensor values (prox_horizontal): ", self.sensor_vals)

        front_prox_horizontal = self.sensor_vals[:5]
        back_prox_horizontal = self.sensor_vals[5:]

        if any([x < self.dist_threshold for x in front_prox_horizontal]):
            if angle < self.val_to_angle(90): # goal is on right side
                await self.turn_right()
            else: # goal is on left side
                await self.turn_left()
            await self.forward()

        if front_prox_horizontal[2] > self.dist_threshold: # front obstacle
            # await self.backward()
            if front_prox_horizontal[1] < front_prox_horizontal[3]: # close to left
                await self.turn_right()
            else: # close to right
                await self.turn_left()

        if any([x > self.dist_threshold for x in front_prox_horizontal[:2]]): # left obstacle
            await self.turn_right()

        if any([x > self.dist_threshold for x in front_prox_horizontal[3:]]): # right obstacle
            await self.turn_left()

        if any([x > self.dist_threshold for x in back_prox_horizontal]): # back obstacle
            await self.forward()
        
        await self.client.sleep(0.1)

        return (self.direction, self.motor_speed)

    async def run(self):
        while True:
            self.sensor_vals = list(self.node['prox.horizontal'])
            angle = self.val_to_angle(60)
            await self.avoid(angle)

if __name__ == "__main__":
    local_naviagtor = LocalNavigator()
    aw(local_naviagtor.run())
    aw(local_naviagtor.node.unlock())