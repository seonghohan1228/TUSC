N_DOTS = 80
MAX_SPEED = 5000  # rpm


file = open('tusc.log', 'r')
Lines = file.readlines()

def print_response(target, current, max_value, n):
    if target <= current:
        for i in range(target - 1):
            print("*", end='')
        print("O", end="")
        for i in range(current - target):
            print("*", end="")
        print
    else:
        for i in range(current):
            print("*", end="")
        for i in range(current):
            print(" ", end="")
        print("O")


# Strips the newline character
for line in Lines:
    line_list = line.strip().split()
    target_speed = line_list[2]
    current_speed = line_list[3]
    print_response(target_speed, current_speed, MAX_SPEED, N_DOTS)
