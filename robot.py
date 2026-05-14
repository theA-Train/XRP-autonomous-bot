# OMH

import os
import wpilib
from drivetrain.subsystem import Drivetrain
from drivetrain.commands import *
import commands2

os.environ["HALSIMXRP_HOST"] = "192.168.42.1"
os.environ["HALSIMXRP_PORT"] = "3540"

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		print("robotInit")
		self.drivetrain = Drivetrain()

	def autonomousInit(self):
		self.drivetrain.right_encoder.reset()
		self.drivetrain.left_encoder.reset()

		self.line_command = Drive_To_Distance(self.drivetrain, 2000, 0.4)
		self.angle_command = Rotate_Drivetrain(self.drivetrain, 60) # creates an equilateral triangle, change it to 90 degrees to make a square or 120 for a pentagon etc.
		self.shape = Shape(self.line_command, self.angle_command).repeatedly()
		print("autonomousInit")

	def autonomousPeriodic(self):
		commands2.CommandScheduler.getInstance().run()

if __name__ == "__main__":
	wpilib.run(MyRobot)