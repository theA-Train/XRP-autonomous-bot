from wpilib import Encoder
from xrp import XRPMotor
import typing
import time

class Drivetrain():
    def __init__(self):
        self.left_motor = XRPMotor(0)
        self.right_motor = XRPMotor(1)
		
        self.right_motor.setInverted(True) 
        
        self.left_encoder = Encoder(4,5)
        self.right_encoder = Encoder(6,7)

        self.wheel_measurement = 7.47

    def drive_distance(self, distance: int):
        self.distance = distance 
        while self.left_encoder.getDistance() < self.distance:
            self.left_motor.set(0.5)
            self.right_motor.set(0.5)
            time.sleep(0.02)
        self.left_motor.stopMotor()
        self.right_motor.stopMotor()

        
        
        
    