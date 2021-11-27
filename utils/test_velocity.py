from tdmclient import ClientAsync, aw
import numpy as np
from scipy.interpolate import interp1d
import math
import time

client = ClientAsync()
node = aw(client.wait_for_node())

aw(node.lock())
aw(node.wait_for_variables())

def motor(l_speed=500, r_speed=500):
    # print("\tSetting speed: ", l_speed, r_speed)
    return {
            "motor.left.target": [l_speed],
            "motor.right.target": [r_speed],
            }

start = time.time()

async def turn():
    await node.set_variables(motor(30, -30))
    
async def run():
    while True:
        end = time.time()
        if end - start >= 10: # 10 seconds
            break
        await turn()
    print("start : ", start)
    print("end : ", end)
    print("time : ", end - start)
    await node.set_variables(motor(0, 0))

# 300, -300
# 3바퀴 + 5도
# 3바퀴 + 10도
# 3바퀴 + 1도

# 200, -200
# 2바퀴 + 34도 = 554
# 2바퀴 + 30도

# 100, -100
# 1바퀴 + 5도
# 345도
# 1바퀴 + 3도
# 1바퀴 + 70도
# 348도
# 1바퀴 + 19도

# 80, -80
# 319도
# 326도
# 325도

# 50, -50
# 222도
# 224도

# 30 -30
# 136도
# 135도

aw(run())
aw(node.unlock())