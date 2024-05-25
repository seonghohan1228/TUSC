N_DOTS = 80
MAX_SPEED = 5000  # rpm
N_HEADER = 3

def print_response(target_value, current_value, max_value, n):
    target = n * target_value / max_value
    current = n * current_value / max_value
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

file = open('tusc.log', 'r')
Lines = file.readlines()


# Strips the newline character
for i, line in enumerate(Lines):
    if i >= 3:
        line_list = line.strip().split()
        target_speed = float(line_list[2])
        current_speed = float(line_list[3])
        print_response(target_speed, current_speed, MAX_SPEED, N_DOTS)
