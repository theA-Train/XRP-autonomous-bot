import os
from enum import Enum
import wpilib
from drivetrain import Drivetrain



os.environ["HALSIMXRP_HOST"] = "192.168.42.1"
os.environ["HALSIMXRP_PORT"] = "3540"

# ---------------- CONSTANTS ----------------
LINE_THRESHOLD   = 0.7    # Reflectance value above which a line is detected (0.0–1.0)
BASE_SPEED       = 0.7    # Drive speed (0.0–1.0)
TILT_ANGLE       = 10.0   # Ramp degree tilt
class State(Enum):
    RAM = 0
    FIND_OPPONENT = 1
    LINE_DETECTED = 2
    TILTED = 3
 
class MyRobot(wpilib.TimedRobot):
 
    def robotInit(self):
        print("robotInit")
        self.drivetrain = Drivetrain()
 
    def autonomousInit(self):
        print("autonomousInit")
        self.drivetrain.reset_encoders()
        self.state = State.FIND_OPPONENT
        self.target_search_angle = 0.0
        self.target_ram_heading = 0.0
        self.escape_timer = wpilib.Timer()
        self.drive_clear_timer = wpilib.Timer()
        self.driving_clear = False   
 # -------------- HELPER FUNCTIONS -------------- 

    def tilt_detection(self):
        if self.drivetrain.get_yaw_angle() > TILT_ANGLE:
            return True
        return False

    def line_front_detected(self):
            """Returns True if either front sensors sees a line."""
            left_front_val = self.drivetrain.front_reflectance.getLeftReflectanceValue()
            right_front_val = self.drivetrain.front_reflectance.getRightReflectanceValue()
            return left_front_val > LINE_THRESHOLD or right_front_val > LINE_THRESHOLD
 
    # def line_rear_detected(self):
            """Returns True if either rear sensors sees a line."""
            left_rear_val = self.drivetrain.get_left_rear_reflectance()
            # right_rear_val = self.drivetrain.get_right_rear_reflectance()
            return left_rear_val > LINE_THRESHOLD or right_rear_val > LINE_THRESHOLD
        
    def opponent_detected(self):
        return self.drivetrain.object_detection()

    def autonomousPeriodic(self):
        
        if self.state != State.LINE_DETECTED and self.line_front_detected():
            left_hit = self.drivetrain.get_left_front_reflectance() > LINE_THRESHOLD
            right_hit = self.drivetrain.get_right_front_reflectance() > LINE_THRESHOLD
            
            if left_hit or right_hit:
                self.line_entry_angle = self.drivetrain.get_gyro_angle()
                
                turn_offset = 135.0 if left_hit else -135.0
                self.target_escape_angle = self.line_entry_angle + turn_offset

        # ---------------- STATE MACHINE ----------------
        if self.line_front_detected():
            self.state = State.LINE_DETECTED
        # print(self.drivetrain.get_left_front_reflectance())

        if self.tilt_detection():
            self.state = State.TILTED

        print(self.drivetrain.get_yaw_angle())
        match(self.state):
            case (State.RAM):
                print("ram")
                self.drivetrain.drive_straight(-BASE_SPEED, self.target_ram_heading)
                if not self.opponent_detected():
                    self.state = State.FIND_OPPONENT
            case (State.FIND_OPPONENT):
                print("finding opponent")
                self.target_search_angle += 2.5 
                self.drivetrain.rotate_drivetrain(self.target_search_angle)
                if self.opponent_detected():
                    self.target_ram_heading = self.drivetrain.get_gyro_angle()
                    self.state = State.RAM
            case (State.LINE_DETECTED):
                print("line detected")
                self.drivetrain.rotate_drivetrain(self.target_escape_angle)
                current_error = self.target_escape_angle - self.drivetrain.get_gyro_angle()
                if abs(current_error) >= 10.0:
                    self.drivetrain.rotate_drivetrain(self.target_escape_angle)
                    self.driving_clear = False
            
                elif not self.line_front_detected():
                    if not self.driving_clear:
                        self.drive_clear_timer.reset()
                        self.drive_clear_timer.start()
                        self.driving_clear = True

                    self.drivetrain.drive_straight(-BASE_SPEED, self.target_escape_angle)

                    if self.drive_clear_timer.get() >= 1.5:
                        self.driving_clear = False
                        self.target_search_angle = self.drivetrain.get_gyro_angle()
                        self.state = State.FIND_OPPONENT

                    else:
                        self.driving_clear = False
                        self.drivetrain.rotate_drivetrain(self.target_escape_angle)
                        
            case (State.TILTED):
                print("titled")
                self.drivetrain.drive_straight(-BASE_SPEED, self.target_ram_heading)
                if not self.tilt_detection():
                    self.state = State.FIND_OPPONENT
 
if __name__ == "__main__":
    wpilib.run(MyRobot)
