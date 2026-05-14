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

		self.line = Drive_To_Distance(self.drivetrain, 5000, 0.4)
		self.angle = Rotate_Drivetrain(self.drivetrain, 60) # change the angle to get the desired equilateral shape :)
		self.shape = Shape_Path(self.line, self.angle).repeatedly()
		self.shape.schedule()
		print("autonomousInit")

	def autonomousPeriodic(self):
		commands2.CommandScheduler.getInstance().run()

if __name__ == "__main__":
	wpilib.run(MyRobot)