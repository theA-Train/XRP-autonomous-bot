import os
import wpilib
from xrp import XRPMotor

os.environ["HALSIMXRP_HOST"] = "192.168.42.1"
os.environ["HALSIMXRP_PORT"] = "3300" #3540

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		print("robotInit")
		# Example: Initialize XRP motors
		self.left_motor = XRPMotor(0)
		self.right_motor = XRPMotor(1)
		self.right_motor.setInverted(True) 

	def autonomousInit(self):
		print("autonomousInit")

	def autonomousPeriodic(self):
		print("autonomousPeriodic")
		# Move forward (speed, rotation) for 2 seconds
		self.left_motor.set(0.5)
		self.right_motor.set(0.5)
		

if __name__ == "__main__":
	wpilib.run(MyRobot)

# from XRPLib.defaults import *

# drivetrain.set_effort(0.5, 0.5)
# print("success")