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
		self.drive_to_ramp_command = Drive_To_Distance(self.drivetrain, 5500, 0.4)
		self.drive_up_ramp_command = Drive_To_Distance(self.drivetrain, 34300, 0.7)
		self.rotate_to_ramp_command = Rotate_Drivetrain(self.drivetrain, 85)
		self.rotate_from_ramp_command = Rotate_Drivetrain(self.drivetrain, 88)
		self.drive_bridge_command = Drive_To_Distance(self.drivetrain, 20000, 0.4)
		self.wait_command = Wait(0.3)
		self.ramp_auto = Ramp_Routine(self.drive_to_ramp_command, self.wait_command, self.rotate_to_ramp_command, self.drive_up_ramp_command, self.rotate_from_ramp_command, self.drive_bridge_command)
		# self.rotate_to_ramp_command.schedule()
		self.ramp_auto.schedule()
		# self.drive_up_ramp_command.schedule()
		print("autonomousInit")

	def autonomousPeriodic(self):
		commands2.CommandScheduler.getInstance().run()

if __name__ == "__main__":
	wpilib.run(MyRobot)