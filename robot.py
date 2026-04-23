import os
import wpilib
from wpilib import Timer
from xrp import XRPMotor
from xrp import XRPGyro
from math import pi

os.environ["HALSIMXRP_HOST"] = "192.168.42.1"
os.environ["HALSIMXRP_PORT"] = "3540" #3540

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		print("robotInit")
		# Example: Initialize XRP motors
		self.left_motor = XRPMotor(0)
		self.right_motor = XRPMotor(1)
		self.right_motor.setInverted(True) 
		self.gyro = XRPGyro()

	def autonomousInit(self):
		print("autonomousInit")


	def autonomousPeriodic(self):
		print("autonomousPeriodic")
		print(self.gyro.getAngle() * (180/pi))
		
if __name__ == "__main__":
	wpilib.run(MyRobot)