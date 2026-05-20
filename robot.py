# OMH

import os
import wpilib
from drivetrain import *
import commands2

os.environ["HALSIMXRP_HOST"] = "192.168.42.1"
os.environ["HALSIMXRP_PORT"] = "3540"

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		print("robotInit")
		self.drivetrain = Drivetrain()

	def autonomousInit(self):
		self.now = Timer.getFPGATimestamp()
		self.drivetrain.right_encoder.reset()
		self.drivetrain.left_encoder.reset()
		self.drive = Drive_To_Distance(self.drivetrain, 0.6)
		self.neutral_turn = Rotate_Drivetrain(self.drivetrain, 60)
		self.line_following = LineFollowing(self.drivetrain, self.drive, self.neutral_turn).repeatedly()
		# self.print_reflectance = PrintReflectance(self.drivetrain)
		self.line_following.schedule()
		# self.print_reflectance.schedule()
		# self.drive.until(lambda: (Timer.getFPGATimestamp() - self.now) >= 2).schedule()
		print("autonomousInit")

	def autonomousPeriodic(self):
		commands2.CommandScheduler.getInstance().run()

if __name__ == "__main__":
	wpilib.run(MyRobot)