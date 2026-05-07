from wpilib import Encoder
import wpimath
from commands2 import Subsystem
from xrp import XRPMotor, XRPGyro
import math

class Drivetrain(Subsystem):
    def __init__(self):
        self.left_motor = XRPMotor(0)
        self.right_motor = XRPMotor(1)
        self.max_effort = 0.8
        self.min_effort = -0.8

        self.right_motor.setInverted(True) 
        
        self.left_encoder = Encoder(4,5)
        self.right_encoder = Encoder(6,7)
        self.gyro = XRPGyro()

    def get_gyro_angle(self):
        # return self.gyro.getRotation2d().degrees() - 360 * math.floor((self.gyro.getRotation2d().degrees() + 180) / 360) this is essentially what angle modulus is doing
        return wpimath.angleModulus(self.gyro.getAngle()) * (180/math.pi)
    
    def clamp_motor_values(self, value: float) -> float:
        return max(min(value, self.max_effort),self.min_effort)
    
    def set_left_motor(self, value:float):
        self.left_motor.set(self.clamp_motor_values(value))

    def set_right_motor(self, value:float):
        self.right_motor.set(self.clamp_motor_values(value))
    



    