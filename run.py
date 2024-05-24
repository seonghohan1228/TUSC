import pigpio
import pygame
import time
import serial
import struct
from packet import Packet
import math


DOWN = 0
UP = 1

LOW = 0
HIGH = 1

STEER = 0
TANK = 1

STOP = 0
RUN = 1


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

	def __init__(self, scalar=10, trim=0):
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


class TUSC:
	LIN_ACT_IN_1_PIN = 22
	LIN_ACT_IN_2_PIN = 27
	STEER_MODE_LED_PIN = 23
	TANK_MODE_LED_PIN = 24
	GEAR_1_LED_PIN = 6
	GEAR_2_LED_PIN = 13
	GEAR_3_LED_PIN = 19
	GEAR_4_LED_PIN = 26

	SCALARS = [20, 40, 60, 80]
	LIN_ACT_COUNT = 100
	DEFAULT_SENSITIVITY = 0.2

	def __init__(self):
		# Setup Pi and actuators
		self.pi = pigpio.pi()
		self.gear = 1
		self.set_scalar(self.gear)
		self.lin_act = LinearActuator(self.pi, self.LIN_ACT_IN_1_PIN, \
								  self.LIN_ACT_IN_2_PIN)
		self.bldc_L = BLDC(scalar=self.scalar, trim=0)
		self.bldc_R = BLDC(scalar=self.scalar, trim=0)
		self.sensitivity = self.DEFAULT_SENSITIVITY
		self.mode = STEER
	
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
	
	def led_control(self):
		if self.mode == STEER:
			self.pi.write(self.STEER_MODE_LED_PIN, HIGH)
			self.pi.write(self.TANK_MODE_LED_PIN, LOW)
		elif self.mode == TANK:
			self.pi.write(self.STEER_MODE_LED_PIN, LOW)
			self.pi.write(self.TANK_MODE_LED_PIN, HIGH)
		
		if self.gear == 1:
			self.pi.write(self.GEAR_1_LED_PIN, HIGH)
			self.pi.write(self.GEAR_2_LED_PIN, LOW)
			self.pi.write(self.GEAR_3_LED_PIN, LOW)
			self.pi.write(self.GEAR_4_LED_PIN, LOW)
		elif self.gear == 2:
			self.pi.write(self.GEAR_1_LED_PIN, HIGH)
			self.pi.write(self.GEAR_2_LED_PIN, HIGH)
			self.pi.write(self.GEAR_3_LED_PIN, LOW)
			self.pi.write(self.GEAR_4_LED_PIN, LOW)
		elif self.gear == 3:
			self.pi.write(self.GEAR_1_LED_PIN, HIGH)
			self.pi.write(self.GEAR_2_LED_PIN, HIGH)
			self.pi.write(self.GEAR_3_LED_PIN, HIGH)
			self.pi.write(self.GEAR_4_LED_PIN, LOW)
		elif self.gear == 4:
			self.pi.write(self.GEAR_1_LED_PIN, HIGH)
			self.pi.write(self.GEAR_2_LED_PIN, HIGH)
			self.pi.write(self.GEAR_3_LED_PIN, HIGH)
			self.pi.write(self.GEAR_4_LED_PIN, HIGH)


	def set_speed(self,input, steer_UD=None, steer_LR=None):
		# LED control
		self.led_control()

		if steer_UD == None and steer_LR == None:
			self.bldc_L.set_speed(input, self.scalar)
			self.bldc_R.set_speed(input, self.scalar)
			return
		
		if self.mode == TANK:
			mapped_input_L = steer_UD
			mapped_input_R = steer_LR
		
		if self.mode == STEER:
			interval = 0.5
			forward = True if steer_UD >= 0 else False
			
			if steer_LR**2 + steer_UD**2 > 0.5**2:
				angle = 2. * math.atan2(-steer_LR, steer_UD)/math.pi if forward else 2. * math.atan2(-steer_LR, -steer_UD)/math.pi
			else:
				angle = 0.
			
			mapped_input_L = max(-1., min(1. ,input + (-interval*angle*self.sensitivity) ))
			mapped_input_R = max(-1., min(1. ,input + (+interval*angle*self.sensitivity) ))
			

		#print(f"mapped_intput_L: {mapped_input_L}")
		#print(f"mapped_input_R: {mapped_input_R}")
		self.bldc_L.set_speed(mapped_input_L, self.scalar)
		self.bldc_R.set_speed(mapped_input_R, self.scalar)
	
	def increase_sensitivity(self):
		self.sensitivity += 0.2
		if self.sensitivity >= 1:
			self.sensitivity = 1
			
	def decrease_sensitivity(self):
		self.sensitivity -= 0.2
		if self.sensitivity <= 0.2:
			self.sensitivity = 0.2
	
	def switch_mode(self):
		if self.mode == STEER:
			self.mode = TANK
		elif self.mode == TANK:
			self.mode = STEER


def main():
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
		ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
		ser.reset_input_buffer()
		tusc = TUSC()
		
		# Main loop
		while True:
			# Get joystick angle (negative is forward)

			speed_input = None
			axis_value_UD = None
			axis_value_LR = None
			axis_value_L = None
			axis_value_R = None

			if tusc.mode == TANK:
				speed_input = None
				axis_value_L = -joystick.get_axis(1)  # Left joystick y
				axis_value_R = -joystick.get_axis(3)  # Right joystick y
				
			elif tusc.mode == STEER:
				axis_value_UD = -joystick.get_axis(ps4_axes["l_stick_v"])
				axis_value_LR = joystick.get_axis(ps4_axes["l_stick_h"])  # Right joystick y
				l2_trigger = -(joystick.get_axis(ps4_axes["l2_trigger"]) + 1.0) /2.
				r2_trigger = -(joystick.get_axis(ps4_axes["r2_trigger"]) + 1.0) /2.
				speed_input = l2_trigger - r2_trigger
			
			# tusc.set_speed(axis_value_L, axis_value_R)
			tusc.set_speed(input=speed_input, steer_UD=axis_value_UD or axis_value_L, steer_LR=axis_value_LR or axis_value_R) 
			# Create and send data packet
			packet = Packet(ser)
			packet.create(RUN, int(tusc.bldc_L.speed), int(tusc.bldc_R.speed))
			packet.send()

			if ser.in_waiting > 0:
				line = ser.readline().decode('utf-8').rstrip()
				print(line)

            # Handle Pygame events
			events = pygame.event.get()
			for event in events:
				# Joystick disconnected
				if event.type == pygame.JOYDEVICEREMOVED:
					print("Joystick disconnected.")
					print("Quiting...")
					tusc.set_speed(0, 0)
					tusc.mode = STEER
					tusc.gear = 1
					packet.create(STOP, 0, 0)
					packet.send()
					tusc.pi.stop()
					pygame.quit()
					exit()

				# Button pressed
				if event.type == pygame.JOYBUTTONDOWN:
					print("Button pressed.")

					# Abort program if PS button is pressed
					if joystick.get_button(ps4_buttons["ps"]):
						print()
						print("Software Kill Switch triggered.")
						print("Quiting...")
						tusc.set_speed(0)
						tusc.mode = STEER
						tusc.gear = 1
						packet.create(STOP, 0, 0)
						packet.send()
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
					if joystick.get_button(ps4_buttons["R stick in"]):
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
		tusc.set_speed(0)
		tusc.mode = STEER
		tusc.gear = 1
		packet.create(STOP, 0, 0)
		packet.send()
		tusc.pi.stop()
		pygame.quit()

	finally:
		tusc.set_speed(0)
		tusc.mode = STEER
		tusc.gear = 1
		packet.create(STOP, 0, 0)
		packet.send()
		tusc.pi.stop()
		pygame.quit()


if __name__ == "__main__":
	main()

