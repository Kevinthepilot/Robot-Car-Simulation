import math

class RobotMovement():
    def __init__(self, robot, timestep):
        self.speed = 5
        self.wheelPeri = 0.04 * 6.28
        self.wheelBase = 0.06*2
        self.robot = robot
        self.timestep = timestep
        
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
        
        
    def move(self, direction, distance=None):
        initial = self.encoder1.getValue()
        self.isGoing = True
        for motor in self.motors:
            motor.setVelocity(self.speed * direction)
        
        if (distance != None):
            rc = abs(distance) / self.wheelPeri * 6.28
            while abs(self.encoder1.getValue() - initial) <= rc:
                if self.robot.step(self.timestep) == -1:
                    break
            self.stop()
        
        self.isGoing = False

    
    def stop(self):
        for motor in self.motors:
            motor.setVelocity(0)
    
    def turn(self, direction, is_90_degree=False):
        #direction == 1 -> turn right
        self.isGoing = True
        initialHeading = self._computeHeading()
        for i, motor in enumerate(self.motors):
            if i % 2 == 1:
                motor.setVelocity(-self.speed * direction)
            else:
                motor.setVelocity(self.speed * direction)
                
        if is_90_degree:
 
            targetHeading = (initialHeading + 90 * -1 * direction) % 360
            while True:
                current = self._computeHeading()
                diff = (targetHeading - current + 180) % 360 - 180 
                if abs(diff) < 5: 
                    break
                if self.robot.step(self.timestep) == -1:
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