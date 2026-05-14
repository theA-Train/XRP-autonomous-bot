import commands2
from wpimath.units import seconds
from wpilib import Timer
from drivetrain.subsystem import Drivetrain
import math

class Drive_To_Distance(commands2.Command):
    '''
    Command to drive to a certain distance at a certain speed. Command will not be interrupted
    Args:
        Subsystem: Drivetrain instance
        Distance: In centimeters
        Speed: Base chassis speed without error correction
    '''
    def __init__(self, subsystem: Drivetrain, distance: int, speed: float):
        super().__init__()
        self.subsystem = subsystem
        self.distance = distance / 7.57
        self.distance_error_threshold = 1
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

        self.past_error = self.error
        self.error = (self.left_encoder_distance - self.right_encoder_distance)
        self.p_term = self.kP*self.error
        # self.d_term = self.kD*((self.error - self.past_error)/(self.dt))

        self.correction = self.p_term

        self.subsystem.differential_drive((self.base_drive + self.correction), (self.base_drive - self.correction))
        print((self.correction), "compensation for right motor")
        print((-self.correction), "compensation for left motor")
		
    def isFinished(self) -> bool:
        # command will NOT be interrupted
        if 0.5*(self.right_encoder_distance + self.left_encoder_distance) >= self.distance:
               return True
        else:
                return False
    
    def end(self, interrupted):
        self.subsystem.differential_drive(0,0)

class Rotate_Drivetrain(commands2.Command):
    '''
    Command to rotate drivetrain (diffy drivetrain so no pivoting.) Command will not be interrupted.

    Args:
        subsystem: Drivetrain instance
        target_angle: float but in degrees
    '''
    def __init__(self, subsystem: Drivetrain, target_angle: float):
        super().__init__()
        self.subsystem = subsystem
        self.target_angle = target_angle
        self.turn = 0.0
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
        self.error = (self.current_angle - self.target_angle)

        self.p_term = self.kP * self.error
        self.integral_error += self.error * self.dt
        self.i_term = self.kI*self.integral_error

        self.turn = (self.p_term + self.i_term)
        self.subsystem.differential_drive(-self.turn, self.turn)
        print(self.turn)

    def isFinished(self) -> bool:
        # command will NOT be interrupted
        if abs(self.target_angle - self.current_angle) < self.error_threshold:
            return True
        else:
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
