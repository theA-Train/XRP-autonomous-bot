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
		self.drivetrain.left_encoder.reset()
		print("autonomousInit")

	def autonomousPeriodic(self):
		self.red = self.drivetrain.right_encoder.getDistance()
		self.led = self.drivetrain.left_encoder.getDistance()
		self.distance_error_threshold = 0.1
		self.error = 0
		self.gyro_angle = self.drivetrain.gyro.getAngle()
		self.kP = 0.025
		if self.red < 10000:
			self.drivetrain.left_motor.set(0.4)
			self.drivetrain.right_motor.set(0.4)
			if self.led > self.red:
				self.error = (self.led - self.red)
				self.drivetrain.right_motor.set(0.4 + (self.kP*(self.error)))
				self.drivetrain.left_motor.set(0.4 - (self.kP*(self.error)))
				print(self.kP*(self.led - self.red), "compensation for right motor")
			if self.led < self.red:
				self.error = (self.red - self.led)
				self.drivetrain.left_motor.set(0.4 + (self.kP*(self.error)))
				self.drivetrain.right_motor.set(0.4 - (self.kP*(self.error)))
				print(self.kP*(self.red - self.led), "compensation for left motor")
			if (self.led - self.red) < self.distance_error_threshold or (self.red - self.led) < self.distance_error_threshold:
				self.drivetrain.left_motor.set(0.4)
				self.drivetrain.right_motor.set(0.4)
		else: 
			self.drivetrain.left_motor.stopMotor()
			self.drivetrain.right_motor.stopMotor()
		


if __name__ == "__main__":
	wpilib.run(MyRobot)