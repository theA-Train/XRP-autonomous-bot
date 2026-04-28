from wpilib import Encoder
import wpilib
from xrp import XRPMotor, XRPGyro
import typing
import time

class Drivetrain():
    def __init__(self):
        self.left_motor = XRPMotor(0)
        self.right_motor = XRPMotor(1)
		
        self.right_motor.setInverted(True) 
        
        self.left_encoder = Encoder(4,5)
        self.right_encoder = Encoder(6,7)
        self.gyro = XRPGyro()

    