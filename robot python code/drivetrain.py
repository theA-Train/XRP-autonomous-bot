import os
from commands2 import Subsystem
import wpilib
from xrp import XRPMotor, XRPGyro, XRPReflectanceSensor, XRPRangefinder

class CustomPIDController():
    """
    Creates a PID controller with the correction logic

    Methods:
        __init__(): Initalizes kP, kI, kD constants
        calculuate(): Outputs the appropriate correction terms, given an error term
    """
    def __init__(self, kP: float, kI: float, kD: float):
        """
        Initalize constants for error terms
        
        :param kP: Proportional constant
        :type kP: float
        :param kI: Integral constant
        :type kI: float
        :param kD: Derivative constant
        :type kD: float
        """
        # PID constants
        self.kP, self.kI, self.kD = kP, kI, kD
        # self.i_limit = i_limit

        # Resetting
        self.integral = 0
        self.initial_time = wpilib.Timer.getFPGATimestamp()
        self.last_error = 0

    def calculate(self, error):
        """
        Pass in the error, outputs the appropriate correction terms
        
        :param error: Distance from target output.
        :returns: The sum of the P, I and D term
        """
        now = wpilib.Timer.getFPGATimestamp()
        dt = now - self.initial_time

        self.last_error = error

        p_out = self.kP * error
        self.integral += error * dt
        i_out = self.kI * self.integral
        d_out = self.kD * ((error - self.last_error)/dt)

        correction = p_out + i_out + d_out
        return correction

class Drivetrain(Subsystem):
    def __init__(self):
        # Front drive
        self.left_front_motor = XRPMotor(0)
        self.right_front_motor = XRPMotor(1)
        self.right_front_motor.setInverted(True)
        
        # Rear Drive
        self.left_rear_motor = XRPMotor(3)
        self.right_rear_motor = XRPMotor(4)
        self.right_rear_motor.setInverted(True)

        # --- Encoders ---
        self.left_front_encoder = wpilib.Encoder(4, 5)
        self.right_front_encoder = wpilib.Encoder(6, 7)
        self.left_rear_encoder = wpilib.Encoder(8, 9)
        self.right_rear_encoder = wpilib.Encoder(10,11)
        
        self.left_front_encoder.setDistancePerPulse(7.47 / 585)
        self.right_front_encoder.setDistancePerPulse(7.47 / 585)
        self.left_rear_encoder.setDistancePerPulse(7.47 / 585)
        self.right_rear_encoder.setDistancePerPulse(7.47 / 585)

        # --- Gyro ---
        self.gyro = XRPGyro()

        # --- Line Sensors ---
        self.front_reflectance = XRPReflectanceSensor()
        self.rear_reflectance = XRPReflectanceSensor()

        # PID controllers
        self.drive_pid = CustomPIDController(0.1, 0, 0)
        self.gyro_pid = CustomPIDController(0.3, 0.0, 0.2)

        # Weighted constant for error fusion
        self.alpha = 0.8

    def set_motor_speeds(self, left_speed: float, right_speed: float):
        self.left_front_motor.set(left_speed)
        self.left_rear_motor.set(left_speed)

        self.right_front_motor.set(right_speed)
        self.right_rear_motor.set(right_speed)

    def drive_straight_fusion(self, base_speed: float, target_heading = 0.0):
        left_avg_dist = (self.left_rear_encoder.getDistance() + self.left_front_encoder.getDistance()) / 2
        right_avg_dist = (self.right_front_encoder.getDistance() + self.left_front_encoder.getDistance()) / 2

        gyro_error = target_heading - self.gyro.getAngle()
        encoder_error = left_avg_dist - right_avg_dist

        fused_error = (self.alpha * gyro_error) + ((1 - self.alpha)* encoder_error)
        correction = self.drive_pid.calculate(fused_error)

        self.set_motor_speeds(base_speed + correction, base_speed - correction)



