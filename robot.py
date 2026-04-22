import os
import wpilib
from wpilib import Timer
from xrp import XRPMotor
from drivetrain.drivetrain import Drivetrain 

os.environ["HALSIMXRP_HOST"] = "192.168.42.1"
os.environ["HALSIMXRP_PORT"] = "3540" #3540

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		print("robotInit")
		# Example: Initialize XRP motors
		self.drivetrain = Drivetrain()

	def autonomousInit(self):
		print("autonomousInit")
		self.now = Timer()
		self.now.reset()
		self.now.start()


	def autonomousPeriodic(self):
		self.drivetrain.drive_distance(100)
		if __name__ == "__main__":
			wpilib.run(MyRobot)