"""robotcar controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Keyboard
from algorithm import Algo
from movement import RobotMovement

robot = Robot()
timestep = int(robot.getBasicTimeStep())


keyboard = Keyboard()
keyboard.enable(timestep)

robot.step(timestep)
robotCar = RobotMovement(robot, timestep)
robotCar.init()
robotCar.encoder1.enable(timestep)
robotCar.encoder2.enable(timestep)
robotCar.compass.enable(timestep)

algo = Algo()
algo.readTest()
algo.a_star()
path = algo.movePath
print(path)

facing = 'U'
for id, dir in enumerate(path):
    i = algo.translate(facing, dir)
    if i == 'U': robotCar.move(1)
    elif i == 'R': 
        robotCar.turn(1)
        robotCar.move(1)
    else: 
        robotCar.turn(-1)
        robotCar.move(1)
    facing = dir

# lastTime = robot.getTime()



# while robot.step(timestep) != -1:

    # cTime = robot.getTime()
    # key = keyboard.getKey()
 
    # if (key == 87): #Go forward
        # if (robotCar.isGoing == False): robotCar.move(0.25, 1)
    # elif key == 83: #Go backward
        # if (robotCar.isGoing == False): robotCar.move(0.25, -1)
    # elif key == 68: #Turn right
        # if (robotCar.isGoing == False): robotCar.turn(1)
    # elif key == 65: #Turn left
        # if (robotCar.isGoing == False): robotCar.turn(-1)
    # else: robotCar.stop()

