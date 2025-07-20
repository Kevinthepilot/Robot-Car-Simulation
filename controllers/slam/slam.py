"""slam controller."""
from controller import Robot
from controller import Keyboard
from movement import RobotMovement
import numpy as np
import math
import matplotlib.pyplot as plt
import time
from bresenham import bresenham


class LidarHandler:
    def __init__(self, robot, timestep, map_width, map_height):
        self.lidar = robot.getLidar("lidar")
        self.lidar.enable(timestep)

        self.lidar_motor = robot.getMotor("lidarMotor")
        self.lidar_motor.setPosition(float("inf"))
        self.lidar_motor.setVelocity(0.0)

        self.lidar_compass = robot.getCompass("lidarCompass")
        self.lidar_compass.enable(timestep)

        self.map_width = map_width
        self.map_height = map_height
        self.map = [[0 for _ in range(map_width)] for _ in range(map_height)]
        self.carStatus = {
            "x": 0,
            "y": 0,
        }


    def _compute_heading(self, values):
        rad = math.atan2(values[0], values[1])  # atan2(x, y)
        deg = math.degrees(rad)
        if deg < 0:
            deg += 360
        return deg  # Heading in degrees [0, 360)

    def update_map(self):
        map_resolution = 0.25  # meters per grid cell
    
        heading_deg = self._compute_heading(self.lidar_compass.getValues())
        heading_rad = math.radians(heading_deg)
    
        ranges = self.lidar.getRangeImage()
        fov = self.lidar.getFov()
        resolution = self.lidar.getHorizontalResolution()
    
        half_fov = fov / 2.0
        angle_step = fov / resolution
    
        x_robot = self.carStatus['x']  # in grid units
        y_robot = self.carStatus['y']  # in grid units
    
        for i, distance in enumerate(ranges):
            if distance == float('inf') or distance < 0.1:
                continue  # Skip bad readings
    
            distance_cells = distance / map_resolution  # convert to grid units
    
            beam_angle = -half_fov + i * angle_step
            total_angle = heading_rad + beam_angle
    
            # Compute hit point in grid units
            x_hit = distance_cells * math.cos(total_angle)
            y_hit = distance_cells * math.sin(total_angle)
    
            map_x = int(x_robot + x_hit)
            map_y = int(y_robot + y_hit)
    
            # Bresenham from robot cell to hit cell
            x_start = int(x_robot)
            y_start = int(y_robot)
    
            if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                for x, y in bresenham(x_start, y_start, map_x, map_y):
                    if 0 <= x < self.map_width and 0 <= y < self.map_height:
                        self.map[y][x] = 0  # Free
                self.map[map_y][map_x] = 1  # Occupied


    def print_map(self):
        x = int(self.carStatus['x'])
        y = int(self.carStatus["y"])
        print(f"x: {x}     y: {y}")
        for i, row in enumerate(self.map):
            for j, col in enumerate(row):
                if i == y and j == x: print(2, end=" ")
                else: print(self.map[i][j], end = " ")
            print()
        print("--- END OF MAP ---\n")
    
    def reset_map(self):
        self.map = self.map = [[0 for _ in range(self.map_width)] for _ in range(self.map_height)]



# create the Robot instance.
robot = Robot()
timestep = int(robot.getBasicTimeStep())
keyboard = Keyboard()
keyboard.enable(timestep)
robot.step(timestep)

#ROBOT
robotCar = RobotMovement(robot, timestep)
robotCar.init()
robotCar.encoder1.enable(timestep)
robotCar.encoder2.enable(timestep)
robotCar.compass.enable(timestep)


lidar = LidarHandler(robot, timestep, 30, 30)
#STATES
initialHeading = robotCar._computeHeading()
pr1 = robotCar.encoder1.getValue()
pr2 = robotCar.encoder2.getValue()
lastState = {
    "x": 15.0,
    "y": 15.0
}

while robot.step(timestep) != -1:
    key = keyboard.getKey()
    lidar.lidar_motor.setVelocity(10)
    lidar.carStatus = lastState
    lidar.update_map()
    #print(lidar.lidar.getRangeImage()[0])
    #print(f"x: {int(lastState['x'])}  y: {int(lastState['y'])}")
    
    en1 = robotCar.encoder1.getValue()
    en2 = robotCar.encoder2.getValue()
    heading = (robotCar._computeHeading() - initialHeading + 360) % 360 
    heading = math.radians(heading)
    dl = (en1 - pr1) / 6.28 * robotCar.wheelPeri
    dr = (en2 - pr2) / 6.28 * robotCar.wheelPeri
    dc = (dl + dr) / 2.0
    
    if abs(dc) > 0:
        lastState["x"] += (dc * math.cos(heading)) / 0.25
        lastState["y"] += (dc * math.sin(heading)) / 0.25
        pr1 = en1
        pr2 = en2


    if (key == 87): #Go forward
        robotCar.move(1)
        
    elif key == 83: #Go backward
        robotCar.move(-1)
    
    elif key == 68: #Turn right
        robotCar.turn(1)
    elif key == 65: #Turn left
        robotCar.turn(-1)
        
    elif key == 67:
        lidar.print_map()
        time.sleep(0.5)

    else: robotCar.stop()

# Enter here exit cleanup code.
