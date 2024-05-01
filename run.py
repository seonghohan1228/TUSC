import pigpio
import pygame
import time


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

		self.set_speed(self.input)

	def set_speed(self, input):
		self.speed = (self.scalar / 100) * input * (100 + self.trim)
		self.set_pwm()

	def set_pwm(self):
		pulsewidth = (self.MAX_PULSEWIDTH - self.MAX_PULSEWIDTH) * \
					 (self.speed / 100) + self.MIN_PULSEWIDTH
		self.pi.set_servo_pulsewidth(self.pwm_pin, pulsewidth)


class TUSC:
	LIN_ACT_IN_1_PIN = 22
	LIN_ACT_IN_2_PIN = 27
	PWM_PIN_L = 13
	PWM_PIN_R = 12
	SCALARS = [20, 40, 60, 80, 100]
	LIN_ACT_COUNT = 6000

	def __init__(self):
		# Setup Pi and actuators
		self.pi = pigpio.pi()
		self.gear = 1
		self.set_scalar(self.gear)
		self.lin_act = LinearActuator(self.pi, self.LIN_ACT_IN_1_PIN, 
								  self.LIN_ACT_IN_2_PIN)
		self.bldc_L = BLDC(self.pi, self.PWM_PIN_L, 
					 scalar=self.scalar, trim=0)
		self.bldc_R = BLDC(self.pi, self.PWM_PIN_R, 
					 scalar=self.scalar, trim=0)
	
	def upshift(self):
		if self.gear > len(self.SCALARS):
			return
		self.gear += 1
		self.set_scalar(self.gear)
	
	def downshift(self):
		if self.gear < 1:
			return
		self.gear -= 1
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
		self.bldc_L = 0
		self.bldc_R = 0

	def set_speed(self, input_L, input_R):
		self.bldc_L.set_speed(input_L)
		self.bldc_R.set_speed(input_R)



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
		tusc = TUSC()
		
		# Main loop
		while True:
			# Get joystick angle (negative is forward)
			axis_value_L = -joystick.get_axis(1)
			axis_value_R = -joystick.get_axis(3)
			
			tusc.set_speed(axis_value_L, axis_value_R)
			
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
					if joystick.get_button(ps4_buttons["left"]):
						tusc.trim("L")
	 				# Trim so that robot steers towards right
					elif joystick.get_button(ps4_buttons["right"]):
						tusc.tirm("R")
					# Trim reset
					elif joystick.get_button(ps4_buttons["share"]):
						tusc.reset_trim()

			if tusc.lin_act.joystick_control == False:
				tusc.lin_act.counter += 1
				if tusc.lin_act.counter >= tusc.LIN_ACT_COUNT:
					tusc.lin_act.stop()
					tusc.lin_act.counter = 0


	except KeyboardInterrupt:
		print("Keyboard interrupt")
		tusc.pi.stop()
		pygame.quit()

	finally:
		tusc.pi.stop()
		pygame.quit()


if __name__ == "__main__":
	main()

