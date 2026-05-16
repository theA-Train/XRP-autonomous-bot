from wpilib import Encoder
import wpimath
from commands2 import Subsystem
from xrp import XRPMotor, XRPGyro, XRPReflectanceSensor
import math
import matplotlib.pyplot as plt

class Drivetrain(Subsystem):
    """
    Drivetrain class that initalizes motors, encoders, and gyro, with set and getter logic to clamp motor values to min and max values
    """
    def __init__(self):
        self.left_motor = XRPMotor(0)
        self.right_motor = XRPMotor(1)
        self.max_effort = 1
        self.min_effort = -1

        self.right_motor.setInverted(True) 
        
        self.left_encoder = Encoder(4,5)
        self.right_encoder = Encoder(6,7)
        self.gyro = XRPGyro()
        self.reflection_sensor = XRPReflectanceSensor()

    def get_gyro_angle(self):
        '''
        Returns angles in the range between -180 and 180 degrees as the standard getAngle() method for XRPgyros are continuous.
        '''
        # return self.gyro.getRotation2d().degrees() - 360 * math.floor((self.gyro.getRotation2d().degrees() + 180) / 360) this is essentially what angle modulus is doing
        return wpimath.angleModulus(self.gyro.getAngle()) * (180/math.pi)
        # return self.gyro.getAngle()
    
    def clamp_motor_values(self, value: float) -> float:
        '''
        Clamps a value to -1 and 1 so motors aren't being set to values higher or lower than that.
        
        :param value: Value being clamped between -1 and 1
        '''
        return max(min(value, self.max_effort),self.min_effort)
    
    def differential_drive(self, Rvalue: float, Lvalue: float):
        """
        Set right and left value of motors, clamped to -1 and 1
        """
        self.right_motor.set(self.clamp_motor_values(Rvalue))
        self.left_motor.set(self.clamp_motor_values(Lvalue))

    def get_left_reflectance(self):
        return self.reflection_sensor.getLeftReflectanceValue()
    
    def get_right_reflectance(self):
        return self.reflection_sensor.getRightReflectanceValue()

    def get_tuple_reflectance(self):
        return (self.reflection_sensor.getRightReflectanceValue(), self.reflection_sensor.getLeftReflectanceValue())

    



    