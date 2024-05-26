import pigpio
import pygame


PWM_PIN_L = 13
PWM_PIN_R = 12
MIN_PULSEWIDTH = 1000
IDLE_PULSEWIDTH = 1500
MAX_PULSEWIDTH = 2000


def map_joystick(input):
	pulsewidth = IDLE_PULSEWIDTH + input * 500
	
	return pulsewidth


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
                
                pulsewidth_L = map_joystick(axis_value_L)
                pulsewidth_R = map_joystick(axis_value_R)

                print(f"Pulsewidth (L): {pulsewidth_L:4.2f}\Pulsewidth (R): {pulsewidth_R:4.2f}")
                
                pi.set_servo_pulsewidth(PWM_PIN_L, pulsewidth_L)
                pi.set_servo_pulsewidth(PWM_PIN_R, pulsewidth_L)
    
    except KeyboardInterrupt:
        print("Keyboard interrupt")
        pi.set_servo_pulsewidth(PWM_PIN_L, IDLE_PULSEWIDTH)
        pi.set_servo_pulsewidth(PWM_PIN_R, IDLE_PULSEWIDTH)
        pi.stop()
        pygame.quit()

    finally:
        pi.set_servo_pulsewidth(PWM_PIN_L, IDLE_PULSEWIDTH)
        pi.set_servo_pulsewidth(PWM_PIN_R, IDLE_PULSEWIDTH)
        pi.stop()


if __name__ == "__main__":
	main()