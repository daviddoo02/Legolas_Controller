#!/usr/bin/env python3
  
import rospy
from std_msgs.msg import String, UInt16, Time, Float32, Int32
from sensor_msgs.msg import Joy

from board import SCL, SDA
import busio

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

import time

class Joy_sub:
	
	def __init__(self):

		sub_topic = 'joy'			

		print("Initializing...")
		print()
		print()

		i2c = busio.I2C(SCL, SDA)
		
		self.pca = PCA9685(i2c)

		self.pca.frequency = 50
	
		self.hip1_servo = servo.Servo(self.pca.channels[0], min_pulse=500, max_pulse=2600, actuation_range=270)
		self.hip2_servo = servo.Servo(self.pca.channels[1], min_pulse=500, max_pulse=2600, actuation_range=270)
		self.thigh_servo = servo.Servo(self.pca.channels[2], min_pulse=500, max_pulse=2600, actuation_range=270)
		self.foreleg_servo = servo.Servo(self.pca.channels[3], min_pulse=500, max_pulse=2600, actuation_range=270)
		self.calf_servo = servo.Servo(self.pca.channels[4], min_pulse=500, max_pulse=2600, actuation_range=270)

		self.reset()

		print("Initializing complete")
		print()
		print()

		self.axes = rospy.Subscriber(sub_topic, Joy , self.move_leg)	
		
		
	def digi_end(self, value, max_deg, min_deg): # Digital End stop
		# if it's at max, only move backward

		eh = 0 # doesn't do anything, just keep from error

		if value == max_deg:
			isatmax = True
			return False, isatmax
		if value == min_deg:
			isatmax = False
			return False, isatmax
		else:
			return True, eh

	def axes_callback(self, msg):
		return msg.axes

	def buttons_callback(self, msg):
		return msg.buttons
	
	def move_forward(self, servo_angle):

		print("Moving Forward")
		servo_angle += 5
		print("Current angle: ", servo_angle)

		return servo_angle
	
	def move_backward(self, servo_angle):

		print("Moving Backward")
		servo_angle -= 5
		print("Current angle: ", servo_angle)

		return servo_angle

# ----------------------------------------------------------------------------------------------------------------------------------------------------------

	
	def reset(self):
	
		self.hip1_angle = 185
		self.hip2_angle = 85
		self.thigh_angle = 65
		self.foreleg_angle = 35
		self.calf_angle = 60
		
		self.hip1_servo.angle = self.hip1_angle
		
		time.sleep(0.1)

		
		self.hip2_servo.angle = self.hip2_angle

		time.sleep(0.1)
		
		
		self.thigh_servo.angle = self.thigh_angle

		time.sleep(0.1)
		
		
		self.foreleg_servo.angle = self.foreleg_angle

		time.sleep(0.1)

		
		self.calf_servo.angle = self.calf_angle

		time.sleep(0.1)


	def move_leg(self, msg):
		axes = self.axes_callback(msg)
		buttons = self.buttons_callback(msg)

		delay = 0.03

		print()

		#Calf
		if buttons[4] != 0 or buttons[5] != 0:
			# Check if in range
			in_range, at_max = self.digi_end(self.calf_angle, 140, 45)

			if in_range:
				if buttons[4] == 1:
					self.calf_angle = self.move_forward(self.calf_angle)
					self.calf_servo.angle = self.calf_angle

					time.sleep(delay)

				elif buttons[5] == 1:
					self.calf_angle = self.move_backward(self.calf_angle)
					self.calf_servo.angle = self.calf_angle

					time.sleep(delay)
			else:
				if at_max:
					if buttons[5] == 1:
						self.calf_angle = self.move_backward(self.calf_angle)
						self.calf_servo.angle = self.calf_angle

						time.sleep(delay)

				elif not at_max:
					if buttons[4] == 1:
						self.calf_angle = self.move_forward(self.calf_angle)
						self.calf_servo.angle = self.calf_angle

						time.sleep(delay)




		# Knee
		if axes[7] != 0:

			in_range, at_max = self.digi_end(self.foreleg_angle, 100, 35)

			if in_range:
				if axes[7] == -1:
					self.foreleg_angle = self.move_forward(self.foreleg_angle)
					self.foreleg_servo.angle = self.foreleg_angle

					time.sleep(delay)

					return

				elif axes[7] == 1:
					self.foreleg_angle = self.move_backward(self.foreleg_angle)
					self.foreleg_servo.angle = self.foreleg_angle
					
					time.sleep(delay)

					return
			else:
				if at_max:
					if axes[7] == 1:
						self.foreleg_angle = self.move_backward(self.foreleg_angle)
						self.foreleg_servo.angle = self.foreleg_angle

						time.sleep(delay)

						return
				elif not at_max:
					if axes[7] == -1:
						self.foreleg_angle = self.move_forward(self.foreleg_angle)
						self.foreleg_servo.angle = self.foreleg_angle

						time.sleep(delay)

						return


		# Hip Pitch
		if axes[6] != 0:
			
			in_range, at_max = self.digi_end(self.thigh_angle, 90, 30)
			
			if in_range:
				if axes[6] == -1:
					self.thigh_angle = self.move_forward(self.thigh_angle)
					self.thigh_servo.angle = self.thigh_angle

					time.sleep(delay)

					return

				elif axes[6] == 1:
					self.thigh_angle = self.move_backward(self.thigh_angle)
					self.thigh_servo.angle = self.thigh_angle

					time.sleep(delay)

					return
			else:
				if at_max:
					if axes[6] == 1:
						self.thigh_angle = self.move_backward(self.thigh_angle)
						self.thigh_servo.angle = self.thigh_angle

						time.sleep(delay)

						return
				elif not at_max:
					if axes[6] == -1:
						self.thigh_angle = self.move_forward(self.thigh_angle)
						self.thigh_servo.angle = self.thigh_angle

						time.sleep(delay)

						return


		#Hip 2
		if buttons[2] != 0 or buttons[1] != 0:
			
			in_range, at_max = self.digi_end(self.hip2_angle, 115, 60)

			if in_range:
				if buttons[2] == 1:
					self.hip2_angle = self.move_forward(self.hip2_angle)
					self.hip2_servo.angle = self.hip2_angle

					time.sleep(delay)

					return

				elif buttons[1] == 1:
					self.hip2_angle = self.move_backward(self.hip2_angle)
					self.hip2_servo.angle = self.hip2_angle

					time.sleep(delay)

					return
			else:
				if at_max:
					if buttons[1] == 1:
						self.hip2_angle = self.move_backward(self.hip2_angle)
						self.hip2_servo.angle = self.hip2_angle

						time.sleep(delay)

						return
				elif not at_max:
					if buttons[2] == 1:
						self.hip2_angle = self.move_forward(self.hip2_angle)
						self.hip2_servo.angle = self.hip2_angle

						time.sleep(delay)

						return


		#Hip 1
		if buttons[3] != 0 or buttons[0] != 0:
			
			in_range, at_max = self.digi_end(self.hip1_angle, 200, 150)

			if in_range:
				if buttons[3] == 1:
					self.hip1_angle = self.move_forward(self.hip1_angle)
					self.hip1_servo.angle = self.hip1_angle

					time.sleep(delay)

					return

				elif buttons[0] == 1:
					self.hip1_angle = self.move_backward(self.hip1_angle)
					self.hip1_servo.angle = self.hip1_angle

					time.sleep(delay)

					return
			else:
				if at_max:
					if buttons[0] == 1:
						self.hip1_angle = self.move_backward(self.hip1_angle)
						self.hip1_servo.angle = self.hip1_angle

						time.sleep(delay)

						return
				elif not at_max:
					if buttons[3] == 1:
						self.hip1_angle = self.move_forward(self.hip1_angle)
						self.hip1_servo.angle = self.hip1_angle

						time.sleep(delay)

						return
					
		if buttons[6] == 1:
			print("Reseting")
			self.reset()
			
			return




if __name__ == '__main__':
	rospy.init_node('joy_subs')

	sub = Joy_sub()

	rospy.spin()
	
	# reset servo position
	sub.reset()

	# Declare this after ros spin

	i2c = busio.I2C(SCL, SDA)

	pca = PCA9685(i2c)

	pca.deinit()
