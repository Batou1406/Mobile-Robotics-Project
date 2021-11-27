# Local Navigation

## LocalNavigator
**Fields**  
- client: Thymio client
- node: Thymio node
- dist_threshold: distance vrom obstacles to start to avoid them (default 1600)
- motor_speed: motor speed of Thymio (default 200)
- sensor_vals: proxy horizontal values [0, 6]
- verbose: whether to print status messge or not
- angle: the current rotated angle
- cumulative_angle: the cumulative rotated angle
- turn_direction: sign for turning, turning right: 1, turning left: -1
- start: start time to rotate
- end: end time to rotate
- time: rotation time
- omega: rotation velocity (degree/sec)


|Degree|speed|velocity (degree/sec)|
|:----:|:---:|:-------------------:|
| 1080 | 300 |         108         |
| 554  | 200 |         55          |
| 370  | 100 |         37          |
| 325  | 80  |         32          |
| 222  | 50  |         22          |
| 135  | 30  |         13          |


**Methods**
- get_outputs(): return the current rotated angle, the cumulative rotated angle and the current motor speed
- val_to_angle(val): convert value into angle
- motor(l_speed, r_speed): set motor speed on Thymio
- print_sensor_values(): print sensor values
- compute_angle(): compute the current rotated angle and the cumulative angle of Thymio.
- compute_motor_speed(): get the proper motor speed and corresponding omega value according to the distance of obstacles
- turn_left(): set (-l_speed, r_speed), and compute rotation time
- turn_right(): set (l_speed, -r_speed), and compute rotation time
- forward(): set (l_speed, r_speed)
- backward(): set (-l_speed, -r_speed)
- avoid(angle): avoid obstacles following 5 categories (No obstacle, Front obstacle, Left obstacle, Right obstacle, Back obstacle)
- run(): run the avoid function


## Avoidance algorithm
- No obstacle
```
if all of sensor values are less than the threshold
    Go forward
```
- Front obstacle
```
if the front middle sensor value is greater than the threshold
    if front_middle_sensor_val > 4000  # too close to the obstacle
        Go backward

    if left_sensor_val - right_sensor_val < -100  # close to the left side
        Turn left
    else if left_sensor_val - right_sensor_val > 100  # close to the right side
        Turn right
    else  # parallelly
        Turn left with probability 0.05
        Turn right with probability 0.95
```
- Left obstacle
```
if any of two front left sensor values is greater than the threshold
    Turn right
```
- Right obstacle
```
if any of two front right sensor values is greater than the threshold
    Turn left
```
- Back obstacle
```
if any of two back sensor values is greater than the threshold
    Go forward
```

## TODO
- [ ] Solve the dead lock.
- [ ] Back to the original path (global path)
- [ ] Find the hyperparameter (dist_threshold)
