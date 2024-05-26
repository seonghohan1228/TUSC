import pigpio
import pygame
import math

### Test verified on pi3 ###

DOWN = 0
UP = 1

LOW = 0
HIGH = 1

STEER = 0
TANK = 1

OFF = 0
ON = 1

DISABLED = 0
ENABLED = 1


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

# Restricts the value of x to the range lower ~ upper
restrict_range = lambda x, lower, upper: max(lower, min(x, upper))

class LinearActuator:
	def __init__(self, pi, input_1_pin, input_2_pin):
		self.pi = pi
		self.in_1_pin = input_1_pin
		self.in_2_pin = input_2_pin
		self.in_1_val = LOW
		self.in_2_val = HIGH
		# Controlled by joystick by default
		self.joystick_control = True

		# Controlled by joystick axis 
		self.joystick_control_by_axis = False
		
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

	def __init__(self, pi, pin, scalar=10):
		self.pi = pi
		self.pin = pin
		self.input = 0
		self.speed = 0  # -100 ~ 100
		self.pulsewidth = self.IDLE_PULSEWIDTH
		self.scalar = scalar  # Percent of Max. PWM

		self.set_speed(self.input, self.scalar)

	def set_speed(self, input, scalar):
		self.scalar = scalar
		self.speed = self.scalar * input
		self.set_pwm()
	
	def set_pwm(self):
		self.pulsewidth = (self.speed / 100) * (self.MAX_PULSEWIDTH - self.IDLE_PULSEWIDTH) + self.IDLE_PULSEWIDTH
		self.pi.set_servo_pulsewidth(self.pin, self.pulsewidth)


class TUSC:
	PWM_PIN_L = 13
	PWM_PIN_R = 12
	LIN_ACT_IN_1_PIN = 17
	LIN_ACT_IN_2_PIN = 27
	STEER_MODE_LED_PIN = 23
	TANK_MODE_LED_PIN = 24
	GEAR_1_LED_PIN = 19
	GEAR_2_LED_PIN = 26
	GEAR_3_LED_PIN = 8
	GEAR_4_LED_PIN = 7

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
		self.bldc_L = BLDC(self.pi, self.PWM_PIN_L, scalar=self.scalar)
		self.bldc_R = BLDC(self.pi, self.PWM_PIN_R, scalar=self.scalar)
		self.sensitivity = self.DEFAULT_SENSITIVITY
		self.mode = STEER
		self.pid = ON
	
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
			interval = 0.5*self.sensitivity
			forward = True if steer_UD >= 0 else False
			
			if steer_LR**2 + steer_UD**2 > 0.5**2:
				angle = 2. * math.atan2(-steer_LR, steer_UD)/math.pi if forward else 2. * math.atan2(-steer_LR, -steer_UD)/math.pi
			else:
				angle = 0.
			
			forth = True if input > 0. else False
			stop = True if input == 0. else False

			input_max = (1. if forth else 0.) if not stop else 1.
			input_min = (0. if forth else -1.) if not stop else -1.

			mapped_input_L = max(input_min, min(input_max,input + (-interval*angle) ))
			mapped_input_R = max(input_min, min(input_max ,input + (+interval*angle) ))			
		
		self.bldc_L.set_speed(mapped_input_L, self.scalar)
		self.bldc_R.set_speed(mapped_input_R, self.scalar)
	
		# LED control
		self.led_control()

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
	
	def print_debug(self):
		print(f"{self.mode}\t{self.gear}\t{self.sensitivity:1.2f}\t\t{self.bldc_L.speed:3.2f}\t{self.bldc_L.pulsewidth:3.2f}\t\t{self.bldc_R.speed:3.2f}\t{self.bldc_R.pulsewidth:3.2f}")


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
			speed_input = None
			axis_value_UD = None
			axis_value_LR = None
			axis_value_L = None
			axis_value_R = None

			# Get joystick angle (negative is forward)
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
			
			tusc.set_speed(input=speed_input, steer_UD=axis_value_UD or axis_value_L, steer_LR=axis_value_LR or axis_value_R)
			tusc.print_debug()

            # Handle Pygame events
			events = pygame.event.get()
			for event in events:
				# Joystick disconnected
				if event.type == pygame.JOYDEVICEREMOVED:
					print("Joystick disconnected.")
					print("Quiting...")
					tusc.mode = STEER
					tusc.gear = 1
					tusc.set_speed(0, 0)
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
						tusc.mode = STEER
						tusc.gear = 1
						tusc.set_speed(0, 0)
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

					# Flipper switches direction if R stick is pressed
					if joystick.get_button(ps4_buttons["cross"]):
						# If linear actuator has stopped, set to extend
						if tusc.lin_act.in_1_val == LOW and tusc.lin_act.in_2_val == LOW:
							tusc.lin_act.extend()
						else:
							tusc.lin_act.flip_direction()
						tusc.lin_act.counter = 0
						tusc.lin_act.joystick_control = True

					# Toggle PID control on/off if L stick is pressed
					if joystick.get_button(ps4_buttons["L stick in"]):
						if tusc.pid == ON:
							tusc.pid = OFF
						else:
							tusc.pid = ON

					# Shifting
					if joystick.get_button(ps4_buttons["L1"]):
						tusc.downshift()
					elif joystick.get_button(ps4_buttons["R1"]):
						tusc.upshift()				

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

			# ********** New Function for Flipper **********

			# Flipper goes up
			if tusc.mode == STEER:
				up_down_flipper = -joystick.get_axis(ps4_axes["r_stick_v"])
				#print(f"up_down : {up_down_flipper}")

				if up_down_flipper > 0.5:
					tusc.lin_act.retract()
					tusc.lin_act.joystick_control_by_axis = True
				elif up_down_flipper < -0.5:
					tusc.lin_act.extend()
					tusc.lin_act.joystick_control_by_axis = True
				

				if up_down_flipper <= 0.5 and up_down_flipper >= -0.5 and tusc.lin_act.joystick_control_by_axis:
					tusc.lin_act.stop()
					tusc.lin_act.joystick_control_by_axis = False

			# ********** New Function for Flipper **********
			
			if tusc.lin_act.joystick_control == False:
				tusc.lin_act.counter += 1
				if tusc.lin_act.counter >= tusc.LIN_ACT_COUNT:
					tusc.lin_act.stop()
					tusc.lin_act.counter = 0
					tusc.lin_act.joystick_control = True


	except KeyboardInterrupt:
		print("Keyboard interrupt")
		tusc.mode = STEER
		tusc.gear = 1
		tusc.set_speed(0, 0)
		tusc.pi.stop()
		pygame.quit()

	finally:
		tusc.set_speed(0)
		tusc.mode = STEER
		tusc.gear = 1
		tusc.set_speed(0, 0)
		tusc.pi.stop()
		pygame.quit()


if __name__ == "__main__":
	main()

