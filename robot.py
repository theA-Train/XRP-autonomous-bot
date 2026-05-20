
import os
import wpilib
from xrp import XRPMotor, XRPGyro
import wpimath
import math
from xrp import XRPReflectanceSensor
import matplotlib.pyplot as plt


os.environ["HALSIMXRP_HOST"] = "192.168.42.1"
os.environ["HALSIMXRP_PORT"] = "3540"
 
 
class MyRobot(wpilib.TimedRobot):
 
    def robotInit(self):
        print("robotInit")

        # # --- Motors ---
        
        # # Front drive
        # self.left_front_motor = XRPMotor(0)
        # self.right_front_motor = XRPMotor(1)
        # self.right_front_motor.setInverted(True)
        
        # # Rear Drive
        # self.left_rear_motor = XRPMotor(3)
        # self.right_rear_motor = XRPMotor(4)

        # # --- Encoders ---
        # self.left_front_encoder = wpilib.Encoder(4, 5)
        # self.right_front_encoder = wpilib.Encoder(6, 7)
        # self.left_rear_encoder = wpilib.Encoder(8, 9)
        # self.right_rear_encoder = wpilib.Encoder(10,11)
        
        # self.left_front_encoder.setDistancePerPulse(7.47 / 585)
        # self.right_front_encoder.setDistancePerPulse(7.47 / 585)
        # self.left_rear_encoder.setDistancePerPulse(7.47 / 585)
        # self.right_rear_encoder.setDistancePerPulse(7.47 / 585)

        # # --- Gyro ---
        # self.gyro = XRPGyro()

        # # --- Line Sensors ---
        # self.reflectance = XRPReflectanceSensor()

        # # --- Timer (used for gyro settle delay) ---
        # self.timer = wpilib.Timer()
 
    def autonomousInit(self):
        print("autonomousInit")
 
        self.left_front_encoder.reset()
        self.right_front_encoder.reset()
        self.gyro.reset()
        self.timer.reset()
        self.timer.start()
 
        # State 0 = drive straight until line
        # State 1 = turn 120 degrees
        # State 2 = done
        self.state = 0
        self.turn_count = 0  # Track how many turns have been completed
 
    def autonomousPeriodic(self):
 
        # ---------------- CONSTANTS ----------------
        LINE_THRESHOLD   = 0.65    # Reflectance value above which a line is detected (0.0–1.0)
                                  # Tune this: white surface reads low (~0.1), dark line reads high (~0.8+)
        TURN_ANGLE       = 117.0  # Target gyro degrees for a 120° turn (adjusted for drift)
        BASE_SPEED       = 0.5    # Drive speed (0.0–1.0)
        TURN_SPEED       = 0.35   # Turn speed
        kP               = 0.1    # Proportional correction gain for driving straight
        GYRO_SETTLE_SEC  = 0.15   # Short delay before reading gyro after a turn starts
 

		# loop variables
        left_front_dist  = self.left_front_encoder.getDistance()
        right_front_dist = self.right_front_encoder.getDistance()
        
		left_rear_dist = self.left_rear_encoder.getDistance()
		right_rear_dist = self.right_rear_encoder.getDistance()
        
 
        # -------------- HELPER FUNCTIONS --------------
 
        def line_detected():
            """Returns True if either sensor sees a line."""
            left_val  = self.reflectance.getLeftReflectanceValue()
            right_val = self.reflectance.getRightReflectanceValue()
            return left_val > LINE_THRESHOLD or right_val > LINE_THRESHOLD
 
        def drive_straight():
            """Drive forward with encoder-based straight correction."""
            right_error = (right_front_dist + right_rear_dist)/2
            right_correction = right_error * kP
            left_error = rig

            self.left_front_motor.set(BASE_SPEED - correction)
            self.right_front_motor.set(BASE_SPEED + correction)
 
        def stop_motors():
            self.left_front_motor.set(0)
            self.right_front_motor.set(0)
 
        def turn_120():
            """
            Spin in place using the gyro.
            Returns True when the target angle has been reached.
            A short settle delay prevents a stale gyro reading from
            triggering an instant exit right after the turn starts.
            """
            if self.timer.get() < GYRO_SETTLE_SEC:
                # Keep turning but don't check angle yet
                self.left_front_motor.set(TURN_SPEED)
                self.right_front_motor.set(-TURN_SPEED)
                return False
 
            try:
                angle_val   = self.gyro.getAngle()
                current_deg = abs(wpimath.angleModulus(angle_val) * (180.0 / math.pi))
            except Exception:
                current_deg = abs(self.gyro.getAngle())
 
            if current_deg < TURN_ANGLE:
                self.left_front_motor.set(TURN_SPEED)
                self.right_front_motor.set(-TURN_SPEED)
                return False
 
            return True  # Turn complete
 
        # ---------------- STATE MACHINE ----------------
 
        if self.state == 0:
            # Drive straight until a line is detected
            if line_detected():
                stop_motors()
                self.left_front_encoder.reset()
                self.right_front_encoder.reset()
                self.gyro.reset()
                self.timer.reset()
                self.timer.start()
                self.state = 1
                print(f"Line detected! Beginning turn {self.turn_count + 1}.")
            else:
                drive_straight()
 
        elif self.state == 1:
            # Turn 120 degrees using the gyro
            if turn_120():
                stop_motors()
                self.left_front_encoder.reset()
                self.right_front_encoder.reset()
                self.gyro.reset()
                self.timer.reset()
                self.timer.start()
                self.turn_count += 1
                print(f"Turn {self.turn_count} complete. Driving forward again.")
                self.state = 0  # Go back to driving straight
 
        # Add a self.state == 2 block here if you want a hard stop
        # after a certain number of turns, e.g.:
        #
        # if self.turn_count >= 3:
        #     self.state = 2
        #
        # elif self.state == 2:
        #     stop_motors()
 
 
if __name__ == "__main__":
    wpilib.run(MyRobot)


