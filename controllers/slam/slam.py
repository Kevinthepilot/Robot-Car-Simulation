"""slam controller."""
from controller import Robot
from controller import Keyboard
from movement import RobotMovement
from algorithm import Algo
import numpy as np
import math
import matplotlib.pyplot as plt
import time
from bresenham import bresenham

fig, ax = plt.subplots()
data = np.random.rand(45, 45) 
img = ax.imshow(data, cmap='Oranges', vmin=0, vmax=1)

class LidarHandler:
    def __init__(self, robot, timestep, map_width, map_height, initHead):
        self.lidar = robot.getLidar("lidar")
        self.lidar.enable(timestep)

        self.lidar_motor = robot.getMotor("lidarMotor")
        self.lidar_motor.setPosition(float("inf"))
        self.lidar_motor.setVelocity(0.0)

        self.lidar_compass = robot.getCompass("lidarCompass")
        self.lidar_compass.enable(timestep)

        self.map_width = map_width
        self.map_height = map_height
        self.map = [[0.0 for _ in range(map_width)] for _ in range(map_height)]
        self.cost_map = [[0.0 for _ in range(map_width)] for _ in range(map_height)]
        self.carStatus = {
            "x": 0,
            "y": 0,
        }
        self.iHead = initHead
        
        self.sub_conf = 0.2
        self.add_conf = 0.5


    def _compute_heading(self, values):
        rad = math.atan2(values[0], values[1])  # atan2(x, y)
        deg = math.degrees(rad)
        if deg < 0:
            deg += 360
        return deg  # Heading in degrees [0, 360)

    def update_map(self):
        map_resolution = 0.25  # meters per grid cell
        heading_deg = self._compute_heading(self.lidar_compass.getValues())
        heading_deg = (heading_deg - self.iHead + 360) % 360
        heading_rad = math.radians(heading_deg)
    
        ranges = self.lidar.getRangeImage()
        fov = self.lidar.getFov()
        resolution = self.lidar.getHorizontalResolution()
    
        half_fov = fov / 2.0
        angle_step = fov / resolution
    
        x_robot = self.carStatus['x']  # in grid units
        y_robot = self.carStatus['y']  # in grid units
    
        for i, distance in enumerate(ranges):
            if distance == float('inf') or distance < 0.2:
                continue  # Skip bad readings
    
            distance_cells = distance / map_resolution  # convert to grid units
    
            beam_angle = -half_fov + i * angle_step
            total_angle = heading_rad + beam_angle
    
            # Compute hit point in grid units
            y_hit = distance_cells * math.cos(total_angle)
            x_hit = distance_cells * math.sin(total_angle)
    
            map_x = int(x_robot + x_hit)
            map_y = int(y_robot + y_hit)
    
            # Bresenham from robot cell to hit cell
            x_start = int(x_robot)
            y_start = int(y_robot)
    
            if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                for x, y in bresenham(x_start, y_start, map_x, map_y):
                    if 0 <= x < self.map_width and 0 <= y < self.map_height:
                        self.map[y][x] -= self.sub_conf  # Free
                        self.map[y][x] = max(-5, self.map[y][x])
                self.map[map_y][map_x] += self.add_conf  # Occupied
                self.map[y][x] = min(5, self.map[y][x])
               
        
        self.update_cost_map(0.3, 3)
    
    def update_cost_map(self, decay_level, cell_num):
        directions = np.arange(-cell_num, cell_num+1)
        n = len(self.map)
        m = len(self.map[0])
        C_max = 1
        
        for i in range(0, n):
            for j in range(0, m):
                if self.map[i][j] < 1: continue
                for dx in directions:
                    for dy in directions:
                        nx = dx + j
                        ny = dy + i
                        if nx >= 0 and nx < m and ny >= 0 and ny < n:
                            dist = (dx**2 + dy**2) ** 0.5
                            cost = min(1, self.map[i][j]) - decay_level * dist
                            self.cost_map[ny][nx] = max(self.cost_map[ny][nx], cost)
                 
        

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
initialHeading = robotCar._computeHeading()

#Lidar
lidar = LidarHandler(robot, timestep, 50, 50, initialHeading)

#STATES
pr1 = robotCar.encoder1.getValue()
pr2 = robotCar.encoder2.getValue()
lastState = {
    "x": 25.0,
    "y": 25.0
}

def update_odometry():
    global pr1, pr2, lastState
    en1 = robotCar.encoder1.getValue()
    en2 = robotCar.encoder2.getValue()
    heading = (robotCar._computeHeading() - initialHeading + 360) % 360 
    heading = math.radians(heading)
    dl = (en1 - pr1) / 6.28 * robotCar.wheelPeri
    dr = (en2 - pr2) / 6.28 * robotCar.wheelPeri
    dc = (dl + dr) / 2.0
    
    if abs(dc) > 0:
        lastState["y"] += (dc * math.cos(heading)) / 0.25
        lastState["x"] += (dc * math.sin(heading)) / 0.25
        pr1 = en1
        pr2 = en2

#Algorithm
algo = Algo()

while robot.step(timestep) != -1:
    key = keyboard.getKey()
    lidar.lidar_motor.setVelocity(10)
    lidar.carStatus = lastState
    lidar.update_map()
    #print(lidar.lidar.getRangeImage()[0])
    #print(f"x: {int(lastState['x'])}  y: {int(lastState['y'])}")
    
    update_odometry()


    if (key == 87): #Go forward
        robotCar.move(1, None)
        
    elif key == 83: #Go backward
        robotCar.move(-1)
    
    elif key == 68: #Turn right
        robotCar.turn(1)
    elif key == 65: #Turn left
        robotCar.turn(-1)
        
    elif key == 67: #Visualizing map
        img.set_data(lidar.cost_map)
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.1)  
    elif key == 91: #Saving the map
        with open("map.txt", "w") as file:
            file.write(f"{lidar.map_height} {lidar.map_width}\n")
            for i in range(len(lidar.map)):
                for j in range(len(lidar.map[0])):
                    file.write(f"{lidar.cost_map[i][j]} ")
                file.write("\n")
            print("Map saved")
    elif key == 61: #Pathfinding
        algo.readTest()
        algo.a_star((25, 24), (15, 33), 0.5)
        path = algo.movePath
        print(path)
        facing = 'U'
        for id, dir in enumerate(path):
            i = algo.translate(facing, dir)
            if i == 'U': robotCar.move(1, 0.25)
            elif i == 'R': 
                robotCar.turn(1, True)
                robotCar.move(1, 0.25)
            else: 
                robotCar.turn(-1, True)
                robotCar.move(1, 0.25)
            facing = dir
            update_odometry()
        print("Destination reached")

    else: robotCar.stop()

# Enter here exit cleanup code.
