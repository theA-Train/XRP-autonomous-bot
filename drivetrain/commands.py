import commands2
from drivetrain.subsystem import Drivetrain

class Drive_To_Distance(commands2.Command):
    def __init__(self, subsystem: Drivetrain, distance: int):
        super().__init__()
        self.subsystem = subsystem
        self.distance = distance
        self.distance_error_threshold = 1.4
        self.error = 0.0
        self.kP = 0.011
        self.addRequirements(self.subsystem)

    def execute(self):
        self.right_encoder_distance = self.subsystem.right_encoder.getDistance()
        self.left_encoder_distance = self.subsystem.left_encoder.getDistance()
        if self.right_encoder_distance < self.distance:
            self.subsystem.left_motor.set(0.4)
            self.subsystem.right_motor.set(0.4)
            if self.left_encoder_distance > self.right_encoder_distance:
                self.error = (self.left_encoder_distance - self.right_encoder_distance)
                self.subsystem.right_motor.set(self.subsystem.clamp_motor((0.4 + (self.kP*(self.error)))))
                self.subsystem.left_motor.set(self.subsystem.clamp_motor(0.4 - (self.kP*(self.error))))
                print(self.kP*(self.left_encoder_distance - self.right_encoder_distance), "compensation for right motor")
            elif self.left_encoder_distance < self.right_encoder_distance:
                self.error = (self.right_encoder_distance - self.left_encoder_distance)
                self.subsystem.left_motor.set(self.subsystem.clamp_motor((0.4 + (self.kP*(self.error)))))
                self.subsystem.right_motor.set(self.subsystem.clamp_motor(0.4 - (self.kP*(self.error))))
                print(self.kP*(self.right_encoder_distance - self.left_encoder_distance), "compensation for left motor")
            # if (self.left_encoder_distance - self.right_encoder_distance) < self.distance_error_threshold or (self.right_encoder_distance - self.left_encoder_distance) < self.distance_error_threshold:
            # 	self.subsystem.left_motor.set(0.4)
            # 	self.subsystem.right_motor.set(0.4)
            # else: 
            #     self.subsystem.left_motor.stopMotor()
		    #     self.subsystem.right_motor.stopMotor()
		
    def isFinished(self) -> bool:
        # command will NOT be interrupted
        if abs(self.right_encoder_distance - self.left_encoder_distance) < self.distance_error_threshold and self.right_encoder_distance >= self.distance:
               return True
        else:
                return False
    
    def end(self, interrupted):
        self.subsystem.left_motor.set(0)
        self.subsystem.right_motor.set(0)

class Rotate_Drivetrain(commands2.Command):
    def __init__(self, subsystem: Drivetrain, target_angle: int):
        super().__init__()
        self.subsystem = subsystem
        self.target_angle = target_angle
        self.error = 0.0
        self.kP = 0.01
        self.error_threshold = 0.7

    def execute(self):
        self.current_angle = self.subsystem.get_gyro_angle()
        print(self.current_angle)
        if self.current_angle - self.target_angle > 0:
            self.error = (self.current_angle - self.target_angle)
            self.subsystem.right_motor.set(0.4 - (self.kP*(1/self.error)))
            self.subsystem.left_motor.set(0.0)
        elif self.current_angle - self.target_angle < 0:
            self.error = (self.target_angle - self.current_angle)
            self.subsystem.left_motor.set(0.4 - (self.kP*(1/self.error)))
            self.subsystem.right_motor.set(0.0)

    def isFinished(self) -> bool:
        if abs(self.target_angle - self.current_angle) < self.error_threshold:
            return True
        else:
            return False

    def end(self, interrupted):
        self.subsystem.left_motor.set(0.0)
        self.subsystem.right_motor.set(0.0)

class Ramp_Routine(commands2.SequentialCommandGroup):
    def addCommands(self, *commands: commands2.Command):
        return super().addCommands(*commands)