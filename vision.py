import cv2
import numpy as np

def goalDetection(map_raw):
    # Values for detecting the color red
    low_red = np.array([150, 100, 50])
    high_red = np.array([200, 255, 255])

    # Convert image to HSV
    map_hsv = cv2.cvtColor(map_raw, cv2.COLOR_BGR2HSV)

    # Apply mask to image to get the goal (red circle)
    mask = cv2.inRange(map_hsv, low_red, high_red)
    map_mask_goal = cv2.bitwise_and(map_raw, map_raw, mask=mask)

    # Convert image to gray
    gray_goal = cv2.cvtColor(map_mask_goal, cv2.COLOR_BGR2GRAY)

    # Detect circles
    circles = cv2.HoughCircles(gray_goal, cv2.HOUGH_GRADIENT,
                               1, 20, param1=100, param2=10,
                               minRadius=20, maxRadius=100)

    # Output
    if circles is None:
        print("Warning: No goal found, take another picture.")
        find_goal(map_raw) # Remove argument when code is modified
    else:
        return np.matrix([[circles[0, 0, 0]], [circles[0, 0, 1]]]) #goal position in x,y form


def robotDetection(map_raw):
    # Values for detecting the color green
    low_green = np.array([10, 100, 50])
    high_green = np.array([130, 255, 255])

    # Convert image to HSV
    map_hsv = cv2.cvtColor(map_raw, cv2.COLOR_BGR2HSV)

    # Apply mask to image to get the goal (red circle)
    mask = cv2.inRange(map_hsv, low_green, high_green)
    map_mask_thymio = cv2.bitwise_and(map_raw, map_raw, mask=mask)

    # Convert image to gray
    gray_thymio = cv2.cvtColor(map_mask_thymio, cv2.COLOR_BGR2GRAY)

    # Detect circles
    circles = cv2.HoughCircles(gray_thymio, cv2.HOUGH_GRADIENT,
                               1, 20, param1=100, param2=10,
                               minRadius=0, maxRadius=100)

    # Output
    if circles.shape[1]<2:
        print("Warning: No thymio found, take another picture.")
        find_thymio(map_raw) # Remove argument when code is modified
    else:
        if circles[0][0][2] > circles[0][1][2]:
            xp,yp,rp = circles[0][1]
            xg,yg,rg = circles[0][0]
        else:
            xp,yp,rp = circles[0][0]
            xg,yg,rg = circles[0][1]

        direction = [xp-xg,yp-yg]

        thymio_x = xg
        thymio_y = yg
        thymio_theta = np.arctan2(direction[1], direction[0])
        return np.matrix([[thymio_x], [thymio_y], [thymio_theta]])


def preprocessing(map_raw):
    # Convert to RGB
    map_rgb = cv2.cvtColor(map_raw, cv2.COLOR_BGR2RGB)

    # Blur Image
    map_blur = cv2.blur(map_rgb,(10,10))

    # Convert to RGB
    map_rgb = cv2.cvtColor(map_blur, cv2.COLOR_BGR2RGB)

    # Apply Bilateral filter
    map_bilateral = cv2.bilateralFilter(map_rgb,5,15,15)

    # Create Binary Image
    map_gray = cv2.cvtColor(map_bilateral, cv2.COLOR_RGB2GRAY)
    _, map_binary = cv2.threshold(map_gray, 70, 255, cv2.THRESH_BINARY_INV)

    # Opening to remove noise
    kernel_morph = np.ones((10,10),np.uint8)
    map_clean = cv2.morphologyEx(map_binary, cv2.MORPH_OPEN, kernel_morph)

    kernel_erode = np.ones((60,60),np.uint8)
    kernel_dilate = np.ones((140,140),np.uint8)
    map_obstacle = cv2.erode(map_clean, kernel_erode, iterations=1)
    map_occupancy = cv2.dilate(map_obstacle, kernel_dilate, iterations=1)

    return map_occupancy


def obstaclesDetection(map_raw, grid_size):

    map_occupancy = preprocessing(map_raw)

    height = grid_size # Size of the grid
    width = round(map_occupancy.shape[1]/map_occupancy.shape[0]*height)

    occupancy_grid = np.zeros((height,width))
    n = round(map_occupancy.shape[0]/height)

    for i in range(height):
        for j in range(width):
            result = np.sum(map_occupancy[n*i:n*(i+1),n*j:n*(j+1)])
            if result > 0 :
                occupancy_grid[height-1-i,j] = 1

    #return occupancy_grid, n
    return occupancy_grid
