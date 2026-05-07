import commands2
from wpimath.units import seconds
from wpilib import Timer
from drivetrain.subsystem import Drivetrain
import math

class Drive_To_Distance(commands2.Command):
    def __init__(self, subsystem: Drivetrain, distance: int):
        super().__init__()
        self.subsystem = subsystem
        self.distance = distance / 7.57
        self.distance_error_threshold = 1
        self.drive = 0.6
        self.past_error = 0.0
        self.error = 0.0
        self.kP = 0.02
        self.kD = 0.01
        self.initial_time = Timer.getFPGATimestamp()
        self.addRequirements(self.subsystem)

    def initialize(self):
        self.subsystem.left_encoder.reset()
        self.subsystem.right_encoder.reset()

    def execute(self):
        self.now = Timer.getFPGATimestamp()
        self.dt = self.now - self.initial_time
        self.right_encoder_distance = self.subsystem.right_encoder.getDistance()
        self.left_encoder_distance = self.subsystem.left_encoder.getDistance()
        if self.right_encoder_distance < self.distance:
            self.subsystem.set_left_motor(self.drive)
            self.subsystem.set_right_motor(self.drive)
            if self.left_encoder_distance > self.right_encoder_distance:
                self.past_error = self.error
                self.error = (self.left_encoder_distance - self.right_encoder_distance)
                self.p_term = self.kP*self.error
                self.d_term = (self.error - self.past_error)/(self.dt)

                self.drive = 0.6 + (self.p_term + self.d_term) 

                self.subsystem.set_right_motor(self.drive)
                self.subsystem.set_left_motor(0.6)
                print((self.drive - 0.6), "compensation for right motor")
            elif self.left_encoder_distance < self.right_encoder_distance:
                self.past_error = self.error
                self.error = (self.right_encoder_distance - self.left_encoder_distance)
                self.p_term = self.kP*self.error
                self.d_term = (self.error - self.past_error)/self.dt

                self.drive = 0.6 + (self.p_term + self.d_term) 

                self.subsystem.set_right_motor(0.6)
                self.subsystem.set_left_motor(self.drive)

                print((self.drive - 0.6), "compensation for left motor")
		
    def isFinished(self) -> bool:
        # command will NOT be interrupted
        if 0.5*(self.right_encoder_distance + self.left_encoder_distance) >= self.distance:
               return True
        else:
                return False
    
    def end(self, interrupted):
        self.subsystem.set_left_motor(0)
        self.subsystem.set_right_motor(0)

class Rotate_Drivetrain(commands2.Command):
    def __init__(self, subsystem: Drivetrain, target_angle: float):
        super().__init__()
        self.subsystem = subsystem
        self.target_angle = target_angle
        self.turn = 0.0
        self.kP = 0.01
        self.kI = (0.012 * (math.pi/180))
        self.error_threshold = 0.3
        self.initial_time = Timer.getFPGATimestamp()
        self.integral_error = 0.0

    def initialize(self):
        self.subsystem.gyro.reset()
        self.current_angle = 0.0

    def execute(self):
        self.current_angle = self.subsystem.get_gyro_angle()
        self.now = Timer.getFPGATimestamp()
        self.dt = self.now - self.initial_time

        print(self.current_angle, "degrees")
        print(self.dt, "dt")
        if self.current_angle - self.target_angle > 0:
            self.error = (self.current_angle - self.target_angle)
            self.p_term = self.kP * self.error
            self.integral_error += self.error * self.dt
            self.i_term = self.kI*self.integral_error
            self.turn = (self.p_term + self.i_term)
            self.subsystem.set_left_motor(self.turn)
            self.subsystem.set_right_motor(-self.turn)
            print(self.turn, "left motor compensation")
        elif self.current_angle - self.target_angle < 0:
            self.error = (self.target_angle - self.current_angle)
            self.p_term = self.kP * self.error
            self.integral_error += self.error * self.dt
            self.i_term = self.kI*self.integral_error
            self.turn = (self.p_term + self.i_term)
            self.subsystem.set_right_motor(self.turn)
            self.subsystem.set_left_motor(-self.turn)
            print(self.turn, "right motor compensation")


    def isFinished(self) -> bool:
        # command will NOT be interrupted
        if abs(self.target_angle - self.current_angle) < self.error_threshold:
            return True
        else:
            return False

    def end(self, interrupted):
        self.subsystem.set_left_motor(0.0)
        self.subsystem.set_right_motor(0.0)

class Wait(commands2.WaitCommand):
    def __init__(self, seconds: float):
        super().__init__(seconds)

class Ramp_Routine(commands2.SequentialCommandGroup):
    def addCommands(self, *commands: commands2.Command):
        return super().addCommands(*commands)