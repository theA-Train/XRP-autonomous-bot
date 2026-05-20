
import os
from enum import Enum
import wpilib
from .drivetrain import Drivetrain


os.environ["HALSIMXRP_HOST"] = "192.168.42.1"
os.environ["HALSIMXRP_PORT"] = "3540"
 
 
class MyRobot(wpilib.TimedRobot):
 
    def robotInit(self):
        print("robotInit")
        self.drivetrain = Drivetrain()
 
    def autonomousInit(self):
        print("autonomousInit")
        self.drivetrain.reset_encoders()
        self.state = 0

    def autonomousPeriodic(self):
 
        # ---------------- CONSTANTS ----------------
        LINE_THRESHOLD   = 0.65    # Reflectance value above which a line is detected (0.0–1.0)
        BASE_SPEED       = 0.5    # Drive speed (0.0–1.0)
        # -------------- HELPER FUNCTIONS --------------
 
        def line_front_detected():
            """Returns True if either front sensors sees a line."""
            left_front_val = self.drivetrain.front_reflectance.getLeftReflectanceValue()
            right_front_val = self.drivetrain.front_reflectance.getRightReflectanceValue()
            return left_front_val > LINE_THRESHOLD or right_front_val > LINE_THRESHOLD
 
        def line_rear_detected():
            """Returns True if either rear sensors sees a line."""
            left_back_val = self.drivetrain.rear_reflectance.getLeftReflectanceValue()
            right_back_val = self.drivetrain.rear_reflectance.getRightReflectanceValue()
            return left_back_val > LINE_THRESHOLD or right_back_val > LINE_THRESHOLD
        
        def opponent_detected():
            return self.drivetrain.object_detection()
        
        # ---------------- STATE MACHINE ----------------
        class State(Enum):
            RAM = 0
            FIND_OPPONENT = 1
            LINE_DETECTED = 2

        self.state = State.FIND_OPPONENT

        def state_machine():
            match(self.state):
                case (State.RAM):
                    if opponent_detected():
                        self.drivetrain.drive_straight(BASE_SPEED)
                    else:
                        self.state = State.FIND_OPPONENT
                case (State.FIND_OPPONENT):
                    self.drivetrain.rotate_drivetrain(BASE_SPEED, 360)
                    if opponent_detected():
                        self.drivetrain.reset_encoders()
                        self.state = State.RAM
                case (State.LINE_DETECTED):
                    if line_front_detected():
                        "hi"

        state_machine()
 
 
if __name__ == "__main__":
    wpilib.run(MyRobot)


