import pigpio
import pygame
import time


UP = 1
DOWN = 0

# Linear Actuator Pins
LIN_ACT_IN_1 = 22
LIN_ACT_IN_2 = 27

PWM_PIN_L = 13
PWM_PIN_R = 12
MIN_PULSEWIDTH = 1000
IDLE_PULSEWIDTH = 1500
MAX_PULSEWIDTH = 2000
POWER_SCALAR = 10  # % of Max. PWM

SCALARS = [10, 20, 30,50,70,100]

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


def map_joystick(input, scalar, trim=0):
	pulsewidth = (scalar * input * 5) * (1 + trim / 100) + IDLE_PULSEWIDTH
	return int(pulsewidth)


def set_pulsewidth(pi, pin, speed, trim=0):
	speed *= (1 + trim / 100)
	pi.set_servo_pulsewidth(pin, speed)

def main():
	# Initialize Pygame and the joystick module
	pygame.init()
	pygame.joystick.init()

	try:
    	# Attempt to initialize the first joystick
		joystick = pygame.joystick.Joystick(0)
		joystick.init()
		print("Joystick detected:", joystick.get_name())
	except pygame.error:
		print("Joystick not detected.")
		return

	try:
		lin_act_in = [0, 1]
		lin_act_ctr = 0
		lin_act_button_ctl = False
		gear = 1
		trim_L = 0
		trim_R = 0
		scalar = POWER_SCALAR

		pi = pigpio.pi()
		pi.write(LIN_ACT_IN_1, lin_act_in[0])
		pi.write(LIN_ACT_IN_2, lin_act_in[1])

		pi.set_servo_pulsewidth(PWM_PIN_R, MIN_PULSEWIDTH)
		time.sleep(0.01)
		pi.set_servo_pulsewidth(PWM_PIN_R, IDLE_PULSEWIDTH)
		time.sleep(0.01)
		
		print("Starting...")

		pi.write(LIN_ACT_IN_1, lin_act_in[0])
		pi.write(LIN_ACT_IN_2, lin_act_in[1])
		
		while True:
            # Handle Pygame events
			events = pygame.event.get()
			
			for event in events:
                # Get joystick angle (negative is forward)
				axis_value_L = -joystick.get_axis(1)
				axis_value_R = -joystick.get_axis(3)
                
				speed_L = map_joystick(axis_value_L, scalar, trim_L)
				speed_R = map_joystick(axis_value_R, scalar, trim_R)
				print(f"Speed (L): {speed_L * (1+trim_L/100)}\tSpeed (R): {speed_R* (1+trim_R/100)}", end="\t")
				
				# Button pressed
				if event.type == pygame.JOYBUTTONDOWN:
					print("Button pressed.")

					# Abort program if PS button is pressed
					if joystick.get_button(ps4_buttons["ps"]):
						print()
						print("Software Kill Switch triggered.")
						print("Quiting...")
						pi.stop()
						pygame.quit()
						exit()
					
					# Flipper goes up
					if joystick.get_button(ps4_buttons["up"]):
						lin_act_in = [1, 0]
						lin_act_ctr += 1
						lin_act_button_ctl = True
					# Flipper goes down
					elif joystick.get_button(ps4_buttons["down"]):
						lin_act_in = [0, 1]
						lin_act_ctr += 1
						lin_act_button_ctl = True

					# Flipper switches direction if L stick is pressed
					if joystick.get_button(ps4_buttons["L stick in"]):
						if lin_act_in == [0, 1]:
							lin_act_in = [1, 0]
						elif lin_act_in == [1, 0]:
							lin_act_in = [0, 1]
						
						lin_act_button_ctl = False
						if lin_act_in == [0, 0]:
							lin_act_in = [0, 1]

					## Shifting
					# Downshift
					if joystick.get_button(ps4_buttons["L1"]):
						gear -= 1
						if gear < 1:
							gear = 1
					# Upshift
					elif joystick.get_button(ps4_buttons["R1"]):
						gear += 1
						if gear > len(SCALARS):
							gear = len(SCALARS)
					
					## Trimming
	 				# Trim so that robot steers towards left
					if joystick.get_button(ps4_buttons["left"]):
						if trim_R < 0:  # reduce the right trim first.
							trim_R += 5
						else:
							trim_L -= 5
						if trim_L == -100:
							trim_L = -100
	 				# Trim so that robot steers towards right
					elif joystick.get_button(ps4_buttons["right"]):
						if trim_L < 0:
							trim_L += 5
						else:
							trim_R -= 5
						if trim_R == -100:
							trim_R = -100
					# Trim reset
					elif joystick.get_button(ps4_buttons["share"]):
						trim_L = 0
						trim_R = 0

					scalar = SCALARS[gear-1]
				

				print(f"Gear {gear}", end="\t")

				if lin_act_in == [0, 1]:
					print("Flipper: DOWN")
				elif lin_act_in == [1, 0]:
					print("Flipper: UP")
				else:
					print("Flipper: Stop")

				pi.set_servo_pulsewidth(PWM_PIN_L, speed_L)
				pi.set_servo_pulsewidth(PWM_PIN_R, speed_R)

				pi.write(LIN_ACT_IN_1, lin_act_in[0])
				pi.write(LIN_ACT_IN_2, lin_act_in[1])

			# Actuated by buttons
			if lin_act_button_ctl:
				# Stop linear actuator motion
				if lin_act_ctr > 3000:
					lin_act_in = [0, 0]
					lin_act_ctr = 0
				# Update linear actuator counter
				if lin_act_ctr != 0:
					lin_act_ctr += 1
					print(lin_act_ctr)

	

	except KeyboardInterrupt:
		print("Keyboard interrupt")
		pi.stop()
		pygame.quit()

	finally:
		pi.stop()
		pygame.quit()

if __name__ == "__main__":
	main()
