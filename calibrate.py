import pigpio
import pygame


PWM_PIN_L = 13
PWM_PIN_R = 12
MIN_PULSEWIDTH = 1000
CENTER_PULSEWIDTH = 1500
MAX_PULSEWIDTH = 2000


def map_joystick(input):
	if input > 0:
		input /= 1
	
	pulsewidth = CENTER_PULSEWIDTH + input * 500
	
	return int(pulsewidth)


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
        pi = pigpio.pi()

        while True:
            # Handle Pygame events
            events = pygame.event.get()
            
            for event in events:
                # Get joystick angle (negative is forward)
                axis_value_L = -(joystick.get_axis(1))
                axis_value_R = -(joystick.get_axis(3))
                
                speed_L = map_joystick(axis_value_L)
                speed_R = map_joystick(axis_value_R)

                print(f"Speed (L): {speed_L}\tSpeed (R): {speed_R}")
                #print(f"Right motor: {axis_value_R:1.3}")
                
                pi.set_servo_pulsewidth(PWM_PIN_L, speed_L)
                pi.set_servo_pulsewidth(PWM_PIN_R, speed_R)
    
    except KeyboardInterrupt:
        print("Keyboard interrupt")
        pi.stop()
        pygame.quit()

    finally:
        pi.stop()


if __name__ == "__main__":
	main()