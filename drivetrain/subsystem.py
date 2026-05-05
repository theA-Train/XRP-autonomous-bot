from wpilib import Encoder
from commands2 import Subsystem
from xrp import XRPMotor, XRPGyro
import math

class Drivetrain(Subsystem):
    def __init__(self):
        self.left_motor = XRPMotor(0)
        self.right_motor = XRPMotor(1)
        self.max_effort = 1.0
        self.min_effort = 0.0
		
        self.right_motor.setInverted(True) 
        
        self.left_encoder = Encoder(4,5)
        self.right_encoder = Encoder(6,7)
        self.gyro = XRPGyro()

    def get_gyro_angle(self):
        return self.gyro.getAngle() * (180/math.pi)
    
    def clamp(self, value: float) -> float:
        return max(min(value, self.max_effort),self.min_effort)




    