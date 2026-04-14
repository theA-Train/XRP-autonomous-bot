import os
import wpilib
from wpilib import Timer
from xrp import XRPMotor

os.environ["HALSIMXRP_HOST"] = "192.168.42.1"
os.environ["HALSIMXRP_PORT"] = "3540" #3540

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		print("robotInit")
		# Example: Initialize XRP motors
		self.left_motor = XRPMotor(0)
		self.right_motor = XRPMotor(1)
		self.right_motor.setInverted(True) 

	def autonomousInit(self):
		print("autonomousInit")
		self.now = Timer()
		self.now.reset()
		self.now.start()

	def autonomousPeriodic(self):
		print("autonomousPeriodic")
		# Move forward (speed, rotation) for 2 seconds
		print(self.now.get())
		for i in range(3):
			time_iteration = 0
			if self.now.get() + time_iteration < 2:
				self.left_motor.set(0.5)
				self.right_motor.set(0.5)
			elif 2 < self.now.get() + time_iteration > 3: 
				self.right_motor.set(0.5)
				self.left_motor.stopMotor()
				time_iteration = 2
		
if __name__ == "__main__":
	wpilib.run(MyRobot)