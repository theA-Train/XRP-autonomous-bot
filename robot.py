import os
import wpilib
from drivetrain.drivetrain import Drivetrain 

os.environ["HALSIMXRP_HOST"] = "192.168.42.1"
os.environ["HALSIMXRP_PORT"] = "3540" #3540

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		print("robotInit")
		# Example: Initialize XRP motors
		self.drivetrain = Drivetrain()

	def autonomousInit(self):
		self.drivetrain.right_encoder.reset()
		print("autonomousInit")

	def autonomousPeriodic(self):
		if self.drivetrain.right_encoder.getDistance() < 1000:
			self.drivetrain.left_motor.set(0.4)
			self.drivetrain.right_motor.set(0.4)
		else: 
			self.drivetrain.left_motor.stopMotor()
			self.drivetrain.right_motor.stopMotor()
		print(self.drivetrain.left_encoder.getDistance())

if __name__ == "__main__":
	wpilib.run(MyRobot)