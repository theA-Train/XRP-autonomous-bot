import os
from commands2 import Subsystem
import wpilib
import wpimath
import math
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

        # Initializing
        self.integral = 0
        self.initial_time = None
        self.last_error = 0

    def calculate(self, error):
        """
        Pass in the error, outputs the appropriate correction terms
        
        :param error: Distance from target output.
        :returns: The sum of the P, I and D term
        """
        now = wpilib.Timer.getFPGATimestamp()
        
        if self.initial_time is None:
            self.initial_time = now
            self.last_error = error
            return self.kP * error
        dt = now - self.initial_time
        if dt <= 0:
            return self.kP * error
        
        p_out = self.kP * error



        self.integral += error * dt
        i_out = self.kI * self.integral

        d_out = self.kD * ((error - self.last_error)/dt)

        self.last_error = error
        self.last_time = now

        correction = p_out + i_out + d_out
        return correction

class Drivetrain(Subsystem):
    def __init__(self):
        # Front drive
        self.left_front_motor = XRPMotor(0)
        self.right_front_motor = XRPMotor(1)
        self.right_front_motor.setInverted(True)
        
        # Rear Drive
        self.left_rear_motor = XRPMotor(2)
        self.right_rear_motor = XRPMotor(3)
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

        # Range finder
        self.range = XRPRangefinder()

        # --- Line Sensors ---
        self.front_reflectance = XRPReflectanceSensor()
        self.rear_reflectance = XRPReflectanceSensor()

        # PID controllers
        self.drive_pid = CustomPIDController(0.01, 0, 0.001)
        self.gyro_pid = CustomPIDController(0.3, 0.0, 0.2)

        # Weighted constant for error fusion
        self.alpha = 0.4

        # Error tolerance constants
        self.error_gyro_tolerance = 3

        # Range finder ram detection constant
        self.ram_range = 0.5
    
    def set_motor_speeds(self, left_speed: float, right_speed: float):
        self.left_front_motor.set(left_speed)
        self.left_rear_motor.set(left_speed)

        self.right_front_motor.set(right_speed)
        self.right_rear_motor.set(right_speed)

    def reset_encoders(self):
        self.left_front_encoder.reset()
        self.left_rear_encoder.reset()

        self.right_front_encoder.reset()
        self.right_rear_encoder.reset()

        self.gyro.reset()

    def reset_gyro(self):
        self.gyro.reset()
    
    def get_gyro_angle(self) -> float:
        return wpimath.angleModulus(self.gyro.getAngle()) * (180/math.pi)
    
    def drive_straight(self, base_speed: float, target_heading = 0.0):
        left_avg_dist = (self.left_rear_encoder.getDistance() + self.left_front_encoder.getDistance()) / 2
        right_avg_dist = (self.right_front_encoder.getDistance() + self.right_rear_encoder.getDistance()) / 2

        gyro_error = target_heading - self.get_gyro_angle()
        encoder_error = left_avg_dist - right_avg_dist

        fused_error = (self.alpha * gyro_error) + ((1 - self.alpha)* encoder_error)
        correction = self.drive_pid.calculate(fused_error)

        self.set_motor_speeds(base_speed + correction, base_speed - correction)
        print(self.get_gyro_angle())

    def rotate_drivetrain(self, target_angle: float):

        gyro_error = target_angle - self.get_gyro_angle()
        turn_speed = self.gyro_pid.calculate(gyro_error)

        # minimum speed logic

        minimum_speed = 0.15 
        if abs(turn_speed) < minimum_speed:
            turn_speed = math.copysign(minimum_speed, turn_speed)
        turn_speed = max(-1.0, min(1.0, turn_speed))

        self.set_motor_speeds(turn_speed, -turn_speed)
        print(self.get_gyro_angle())

    def object_detection(self) -> bool:
        return self.range.getDistance() < self.ram_range