import os
import wpilib
from drivetrain.subsystem import Drivetrain
from drivetrain.commands import Drive_To_Distance, Rotate_Drivetrain, Ramp_Routine
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
		self.drive_command = Drive_To_Distance(self.drivetrain, 10000)
		self.rotate_command = Rotate_Drivetrain(self.drivetrain, 90)
		self.rotate_command.schedule()
		print("autonomousInit")

	def autonomousPeriodic(self):
		commands2.CommandScheduler.getInstance().run()

if __name__ == "__main__":
	wpilib.run(MyRobot)