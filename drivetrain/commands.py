from typing import Callable, Dict, Hashable
import commands2
from wpimath.units import seconds
from wpilib import Timer
from drivetrain.subsystem import Drivetrain
import math
import csv

class Drive_To_Distance(commands2.Command):
    '''
    Command to drive to a certain distance at a certain speed. Command will be interrupted
    Args:
        Subsystem: Drivetrain instance
        Distance: In centimeters
        Speed: Base chassis speed without error correction
    '''
    def __init__(self, subsystem: Drivetrain, speed: float):
        super().__init__()
        self.subsystem = subsystem
        self.base_drive = speed
        self.past_error = 0.0
        self.left_bias = 1
        self.left_error_bias = 1
        self.right_bias = 1
        self.error = 0.0
        self.kP = 0.01
       #  self.kD = 0.001

        self.initial_time = Timer.getFPGATimestamp()
        self.addRequirements(self.subsystem)

    def initialize(self):
        self.subsystem.differential_drive(0,0)
        self.subsystem.left_encoder.reset()
        self.subsystem.right_encoder.reset()

    def execute(self):
        self.now = Timer.getFPGATimestamp()
        self.dt = self.now - self.initial_time
        self.initial_time = self.now
        self.right_encoder_distance = self.subsystem.right_encoder.getDistance()
        self.left_encoder_distance = self.subsystem.left_encoder.getDistance()

        # self.past_error = self.error
        self.error = (self.left_encoder_distance - self.right_encoder_distance)
        self.p_term = self.kP*self.error
        # self.d_term = self.kD*((self.error - self.past_error)/(self.dt))

        self.correction = self.p_term

        self.subsystem.differential_drive((self.base_drive + self.correction), (self.base_drive - self.correction))
        print((self.correction), "compensation for right motor")
        print((-self.correction), "compensation for left motor")
		
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted):
        print('end')

class Rotate_Drivetrain(commands2.Command):
    '''
    Command to rotate drivetrain (diffy drivetrain so no pivoting.) Command will be interrupted.

    Args:
        subsystem: Drivetrain instance
        target_angle: float but in degrees
    '''
    def __init__(self, subsystem: Drivetrain, turn: float) # target_angle: float
        super().__init__()
        self.subsystem = subsystem
        # self.target_angle = target_angle
        self.turn = turn
        self.kP = 0.03
        self.kI = (0.015 * (math.pi/180))
        self.error_threshold = 1.5
        self.initial_time = Timer.getFPGATimestamp()
        self.integral_error = 0.0

    def initialize(self):
        self.subsystem.gyro.reset()
        self.current_angle = self.subsystem.get_gyro_angle()

    def execute(self):
        self.current_angle = self.subsystem.get_gyro_angle()
        self.now = Timer.getFPGATimestamp()
        self.dt = self.now - self.initial_time
        self.initial_time = self.now

        print(self.current_angle, "degrees")
        print(self.dt, "dt")
       #self.error = (self.current_angle - self.target_angle)

       # self.p_term = self.kP * self.error
        # self.integral_error += self.error * self.dt
        # self.i_term = self.kI*self.integral_error

       # self.turn = (self.p_term + self.i_term)
        self.subsystem.differential_drive(-self.turn, self.turn)
        print(self.turn)

    def isFinished(self) -> bool:
        """
       command will not be interrupted 
        """
        return False

    def end(self, interrupted):
        self.subsystem.differential_drive(0,0)

class Wait(commands2.WaitCommand):
    '''
    Command that pauses the scheduler without alerting watchdog
    Args:
        Seconds: float
    '''
    def __init__(self, seconds: float):
        super().__init__(seconds)

class PrintReflectance(commands2.Command):
    def __init__(self, subsystem: Drivetrain):
        super().__init__()
        self.subsystem = subsystem
        self.data_r_values = []
        self.data_l_values = []

    def execute(self):
        # print(self.subsystem.get_tuple_reflectance())
        self.data_r_values.append(self.subsystem.get_tuple_reflectance()[0])
        self.data_l_values.append(self.subsystem.get_tuple_reflectance()[1])
        print()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        with open('leftdatavalues', 'w', newline = '') as file:
            writer = csv.writer(file)
            writer.writerows([self.data_l_values])

        with open('rightdatavalues', 'w', newline ='') as file: 
            writer = csv.writer(file)
            writer.writerows([self.data_r_values])

class LineFollowing(commands2.SelectCommand):
    def __init__(self, subsystem: Drivetrain, drive: Drive_To_Distance, turn: Rotate_Drivetrain):
        self.high_reflectance = 0.7
        super().__init__(
            {
                True: drive.repeatedly(),
                False: turn.repeatedly()
            },
            lambda: subsystem.get_left_reflectance() < self.high_reflectance and subsystem.get_right_reflectance() < self.high_reflectance
        )
        self.addRequirements(subsystem)