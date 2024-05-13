import pigpio
import pygame
import time
import serial
import struct
import math


DOWN = 0
UP = 1

LOW = 0
HIGH = 1


ps4_buttons = {
	"cross": 0,
	"circle": 1,
	"square": 2,
	"triangle": 3,
	"share": 4,
	"ps": 5,
	"options": 6,
	"L stick in": 7,
	"R stick in": 8,
	"L1": 9,
	"R1": 10,
	"up": 11,
	"down": 12,
	"left": 13,
	"right": 14,
	"touchpad": 15
}

ps4_axes = {
	"l_stick_h": 0,
	"l_stick_v": 1,
	"r_stick_h": 2,
	"r_stick_v": 3,
	"l2_trigger": 4,
	"r2_trigger": 5,
}


class LinearActuator:
	def __init__(self, pi, input_1_pin, input_2_pin):
		self.pi = pi
		self.in_1_pin = input_1_pin
		self.in_2_pin = input_2_pin
		self.in_1_val = LOW
		self.in_2_val = HIGH
		# Controlled by joystick by default
		self.joystick_control = True
		self.counter = 0
		self.set_pins()

	def flip_direction(self):
		# Prevent swapping LOW-LOW and HIGH-HIGH
		if self.in_1_val != self.in_2_val:
			temp = self.in_1_val
			self.in_1_val = self.in_2_val
			self.in_2_val = temp
		self.set_pins()

	def retract(self):
		self.in_1_val = HIGH
		self.in_2_val = LOW
		self.set_pins()

	def extend(self):
		self.in_1_val = LOW
		self.in_2_val = HIGH
		self.set_pins()
	
	def stop(self):
		self.in_1_val = LOW
		self.in_2_val = LOW
		self.set_pins()

	def set_pins(self):
		# Sets pin values
		self.pi.write(self.in_1_pin, self.in_1_val)
		self.pi.write(self.in_2_pin, self.in_2_val)


class BLDC:
	MIN_PULSEWIDTH = 1000
	IDLE_PULSEWIDTH = 1500
	MAX_PULSEWIDTH = 2000

	def __init__(self, pi, pwm_pin, scalar=10, trim=0):
		self.pi = pi
		self.pwm_pin = pwm_pin
		self.input = 0
		self.speed = 0  # 0 ~ 100
		self.scalar = scalar  # Percent of Max. PWM
		self.trim = trim  # Percent
		
		if self.scalar < 0 or self.scalar > 100:
			print("Scalar outside range. Exiting...")
			exit()

		if self.trim < -100:
			print("Trim value below minimum value. Exiting...")
			exit()

		self.set_speed(self.input, self.scalar)

	def set_speed(self, input, scalar):
		self.scalar = scalar
		self.speed = (self.scalar / 100) * input * (100 + self.trim)
		self.set_pwm()

	def set_pwm(self):
		pulsewidth = (self.MAX_PULSEWIDTH - self.IDLE_PULSEWIDTH) * \
					 (self.speed / 100) + self.IDLE_PULSEWIDTH
		self.pi.set_servo_pulsewidth(self.pwm_pin, pulsewidth)


class TUSC:
	LIN_ACT_IN_1_PIN = 22
	LIN_ACT_IN_2_PIN = 27
	PWM_PIN_L = 13
	PWM_PIN_R = 12
	SCALARS = [10, 15, 20, 40, 80]
	LIN_ACT_COUNT = 1000
	DEFAULT_SENSITIVITY = 0.2

	def __init__(self):
		# Setup Pi and actuators
		self.pi = pigpio.pi()
		self.gear = 1
		self.set_scalar(self.gear)
		self.lin_act = LinearActuator(self.pi, self.LIN_ACT_IN_1_PIN, \
								  self.LIN_ACT_IN_2_PIN)
		self.bldc_L = BLDC(self.pi, self.PWM_PIN_L, \
					 scalar=self.scalar, trim=0)
		self.bldc_R = BLDC(self.pi, self.PWM_PIN_R, \
					 scalar=self.scalar, trim=0)
		self.sensitivity = self.DEFAULT_SENSITIVITY
		self.mode = "steer"
	
	def upshift(self):
		self.gear += 1
		if self.gear > len(self.SCALARS):
			self.gear = len(self.SCALARS)
		self.set_scalar(self.gear)
	
	def downshift(self):
		self.gear -= 1
		if self.gear < 1:
			self.gear = 1
		self.set_scalar(self.gear)
	
	def set_scalar(self, gear):
		self.scalar = self.SCALARS[gear - 1]
	
	def trim(self, dir):
		# Lower left motor PWM so robot steers to the left
		if dir == "L":
			# When trimming, make sure the other motor trim is 0
			# If not, reduce trim on the other motor first
			if self.bldc_R.trim == 0:
				self.bldc_L.trim -= 5
			else:
				self.bldc_R.trim += 5
		elif dir == "R":
			if self.bldc_L.trim == 0:
				self.bldc_R.trim -= 5
			else:
				self.bldc_L.trim += 5
		else:
			print("Wrong trim direction. Exiting...")
			exit()
	
	def reset_trim(self):
		self.bldc_L.trim = 0
		self.bldc_R.trim = 0

	def set_speed(self,input, steer_UD=None, steer_LR=None):

		if steer_UD == None or steer_LR == None:
			self.bldc_L.set_speed(input, self.scalar)
			self.bldc_R.set_speed(input, self.scalar)
			return
		
		
		# if self.mode == "tank":
		# 	mapped_input_L = input_L
		# 	mapped_input_R = input_R
		
		if self.mode == "steer":
			mapped_input_L = steer_UD + self.sensitivity * steer_LR
			mapped_input_R = steer_UD - self.sensitivity * steer_LR
			if mapped_input_L < -1:
				mapped_input_L = -1
			if mapped_input_R < -1:
				mapped_input_R = -1
			if mapped_input_L > 1:
				mapped_input_L = 1
			if mapped_input_R > 1:
				mapped_input_R = 1
		
		if self.mode == "tank":
			# mapped_input_L = input_L
			# mapped_input_R = input_R
			# (x,y) = (steer_LR, steer_UD)

			
			interval = 0.3
			forward = True if steer_UD >= 0 else False
			angle = 2. * math.atan2(steer_LR, -steer_UD)/math.pi if forward else 2. * math.atan2(-steer_LR, steer_UD)/math.pi
			
			# mapped_input_L = max(-1., min(1. ,input + (-interval*angle if angle<0 else interval*angle) * (1 if forward else -1)))
			# mapped_input_R = max(-1., min(1. ,input + (+interval*angle if angle<0 else -interval*angle) * (1 if forward else -1)))

			mapped_input_L = max(-1., min(1. ,input + (-interval*angle) * (1 if forward else -1)))
			mapped_input_R = max(-1., min(1. ,input + (+interval*angle) * (1 if forward else -1)))
			

					
		self.bldc_L.set_speed(mapped_input_L, self.scalar)
		self.bldc_R.set_speed(mapped_input_R, self.scalar)
	
	def increase_sensitivity(self):
		self.sensitivity += 0.1
		if self.sensitivity >= 1:
			self.sensitivity = 1
			
	def decrease_sensitivity(self):
		self.sensitivity -= 0.1
		if self.sensitivity <= 0.1:
			self.sensitivity = 0.1
	
	def switch_mode(self):
		if self.mode == "steer":
			self.mode = "tank"
		elif self.mode == "tank":
			self.mode = "steer"


def main():
	ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
	# Initialize Pygame and the joystick module
	pygame.init()
	pygame.joystick.init()

	# Attempt to initialize the first joystick
	try:
		joystick = pygame.joystick.Joystick(0)
		joystick.init()
		print("Joystick detected:", joystick.get_name())
	except pygame.error:
		print("Joystick not detected.")
		return

	# Run TUSC
	try:
		tusc = TUSC()
		
		# Main loop
		while True:
			# Get joystick angle (negative is forward)

			speed_input = None
			axis_value_UD = None
			axis_value_LR = None
			axis_value_L = None
			axis_value_R = None

			if tusc.mode == "steer":
				speed_input = None
				axis_value_L = -joystick.get_axis(1)
				axis_value_R = joystick.get_axis(2)  # Right joystick x
				
			elif tusc.mode == "tank":
				axis_value_UD = -joystick.get_axis(ps4_axes["l_stick_v"])
				axis_value_LR = -joystick.get_axis(ps4_axes["l_stick_h"])  # Right joystick y
				l2_trigger = (joystick.get_axis(ps4_axes["l2_trigger"]) + 1.0) /2.
				r2_trigger = (joystick.get_axis(ps4_axes["r2_trigger"]) + 1.0) /2.
			
				speed_input = l2_trigger - r2_trigger
			
			# tusc.set_speed(axis_value_L, axis_value_R)
			tusc.set_speed(input=speed_input, steer_UD=axis_value_UD or axis_value_L, steer_LR=axis_value_LR or axis_value_R) 
			ser.write(struct.pack('>BB', int(tusc.bldc_L.speed) + 100, int(tusc.bldc_R.speed) + 100))

            # Handle Pygame events
			events = pygame.event.get()
			for event in events:
				# Button pressed
				if event.type == pygame.JOYBUTTONDOWN:
					print("Button pressed.")

					# Abort program if PS button is pressed
					if joystick.get_button(ps4_buttons["ps"]):
						print()
						print("Software Kill Switch triggered.")
						print("Quiting...")
						# tusc.set_speed(0, 0)
						tusc.set_speed(0)
						ser.write(struct.pack('>BB', 0 + 100, 0 + 100))
						tusc.pi.stop()
						pygame.quit()
						exit()
					
					# Flipper goes up
					if joystick.get_button(ps4_buttons["up"]):
						tusc.lin_act.retract()
						tusc.lin_act.counter = 0  # Reset counter
						tusc.lin_act.joystick_control = False
					# Flipper goes down
					elif joystick.get_button(ps4_buttons["down"]):
						tusc.lin_act.extend()
						tusc.lin_act.counter = 0  # Reset counter
						tusc.lin_act.joystick_control = False

					# Flipper switches direction if L stick is pressed
					if joystick.get_button(ps4_buttons["L stick in"]):
						# If linear actuator has stopped, set to extend
						if tusc.lin_act.in_1_val == LOW and tusc.lin_act.in_2_val == LOW:
							tusc.lin_act.extend()
						else:
							tusc.lin_act.flip_direction()
						tusc.lin_act.counter = 0
						tusc.lin_act.joystick_control = True

					# Shifting
					if joystick.get_button(ps4_buttons["L1"]):
						tusc.downshift()
					elif joystick.get_button(ps4_buttons["R1"]):
						tusc.upshift()
					
					## Trimming
	 				# Trim so that robot steers towards left
					if joystick.get_button(ps4_buttons["square"]):
						tusc.trim("L")
	 				# Trim so that robot steers towards right
					elif joystick.get_button(ps4_buttons["circle"]):
						tusc.trim("R")
					# Trim reset
					elif joystick.get_button(ps4_buttons["options"]):
						tusc.reset_trim()
					
					## Sensitivity
					if joystick.get_button(ps4_buttons["left"]):
						tusc.decrease_sensitivity()
					elif joystick.get_button(ps4_buttons["right"]):
						tusc.increase_sensitivity()
					elif joystick.get_button(ps4_buttons["share"]):
						tusc.sensitivity = tusc.sensitivity
					
					## Mode
					if joystick.get_button(ps4_buttons["touchpad"]):
						tusc.switch_mode()

			if tusc.lin_act.joystick_control == False:
				tusc.lin_act.counter += 1
				if tusc.lin_act.counter >= tusc.LIN_ACT_COUNT:
					tusc.lin_act.stop()
					tusc.lin_act.counter = 0
					tusc.lin_act.joystick_control = True


	except KeyboardInterrupt:
		print("Keyboard interrupt")
		ser.write(struct.pack('>BB', 0 + 100, 0 + 100))
		# tusc.set_speed(0, 0)
		tusc.set_speed(0)
		tusc.pi.stop()
		pygame.quit()

	finally:
		# tusc.set_speed(0, 0)
		tusc.set_speed(0)
		ser.write(struct.pack('>BB', 0 + 100, 0 + 100))
		tusc.pi.stop()
		pygame.quit()


if __name__ == "__main__":
	main()

