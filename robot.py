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

	def is_path_clear(self):
		# return True if the path ahead is clear (range > 0.4 meters).
		return self.drivetrain.get_range() > 0.4

	def autonomousInit(self):
		self.drivetrain.right_encoder.reset()
		self.drivetrain.left_encoder.reset()

		# base commands for driving and rotating the drivetrain
		self.drive_command = Drive_To_Distance(self.drivetrain, 500, 0.4)
		# rotate 40 degrees from current heading
		self.rotate_command = Rotate_Drivetrain(self.drivetrain, self.drivetrain.get_gyro_angle() + 40)


		# drive until you bump / detect an obstacle within 0.4 meters
		if self.is_path_clear():
			self.drive_command.schedule()
		else:
			# rotate repeatedly until you no longer detect a wall
			self.rotate_detection = self.rotate_command.repeatedly().until(self.is_path_clear)
			self.rotate_detection.schedule()
		print("autonomousInit")

	def autonomousPeriodic(self):
		commands2.CommandScheduler.getInstance().run()

if __name__ == "__main__":
	wpilib.run(MyRobot)