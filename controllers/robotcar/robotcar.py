"""robotcar controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Keyboard
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())

class RobotMotor():
    def __init__(self):
        self.speed = 5
        self.wheelPeri = 0.04 * 6.28
        
        #Motors
        self.motor1 = robot.getMotor("wheel1")
        self.motor2 = robot.getMotor("wheel2")
        self.motor3 = robot.getMotor("wheel3")
        self.motor4 = robot.getMotor("wheel4")
        self.motors = [self.motor1, self.motor2, self.motor3, self.motor4]
        
        #Encoders
        self.encoder1 = robot.getPositionSensor("encoder1")
        self.encoder2 = robot.getPositionSensor("encoder2")
        
        #Compass
        self.compass = robot.getCompass("compass")
        
        #Flags
        self.isGoing = False
        
    def init(self):
        for motor in self.motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
        
    def move(self, distance, direction):
        initial = self.encoder1.getValue()
        rc = abs(distance) / self.wheelPeri * 6.28
        self.isGoing = True
        for motor in self.motors:
            motor.setVelocity(self.speed * direction)
    
        while abs(self.encoder1.getValue() - initial) <= rc:
            if robot.step(timestep) == -1:
                break
        self.stop()
        self.isGoing = False

    
    def stop(self):
        for motor in self.motors:
            motor.setVelocity(0)
    
    def turn(self, direction):
        #direction == 1 -> turn right
        self.isGoing = True
        initialHeading = self._computeHeading()
        targetHeading = (initialHeading + 90 * -1 * direction) % 360
    
        for i, motor in enumerate(self.motors):
            if i % 2 == 1:
                motor.setVelocity(-self.speed * direction)
            else:
                motor.setVelocity(self.speed * direction)
    
        while True:
            current = self._computeHeading()
            diff = (targetHeading - current + 180) % 360 - 180 
            if abs(diff) < 2: 
                break
            if robot.step(timestep) == -1:
                break
    
        self.stop()
        self.isGoing = False
    
    def _computeHeading(self):
        values = self.compass.getValues()
        rad = math.atan2(values[0], values[1])  # atan2(x, y)
        deg = math.degrees(rad)
        if deg < 0:
            deg += 360
        return deg  # Heading in degrees [0, 360)
        
# create the Robot instance.

# get the time step of the current world.


# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller

keyboard = Keyboard()
keyboard.enable(timestep)


robotCar = RobotMotor()

robotCar.init()
robotCar.encoder1.enable(timestep)
robotCar.encoder2.enable(timestep)
robotCar.compass.enable(timestep)


lastTime = robot.getTime()
while robot.step(timestep) != -1:

    cTime = robot.getTime()
    key = keyboard.getKey()
    direction = robotCar.compass.getValues()
 
    if (key == 87): #Go forward
        if (robotCar.isGoing == False): robotCar.move(0.25, 1)
    elif key == 83: #Go backward
        if (robotCar.isGoing == False): robotCar.move(0.25, -1)
    elif key == 68: #Turn right
        if (robotCar.isGoing == False): robotCar.turn(1)
    elif key == 65: #Turn left
        if (robotCar.isGoing == False): robotCar.turn(-1)
    else: robotCar.stop()

