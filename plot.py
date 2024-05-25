N_DOTS = 80
MAX_SPEED = 5000  # rpm
N_HEADER = 3

def print_response(timestamp, target_value, current_value, max_value, n):
    print(timestamp, end=' ')
    target = int(n * target_value / max_value)
    current = int(n * current_value / max_value)
    if target <= current:
        for i in range(target - 1):
            print("*", end='')
        print("O", end="")
        for i in range(current - target):
            print("*", end="")
        print()
    else:
        for i in range(current):
            print("*", end="")
        for i in range(target - current - 1):
            print(" ", end="")
        print("O")

file = open('tusc.log', 'r')
Lines = file.readlines()


# Strips the newline character
for i, line in enumerate(Lines):
    if i >= 3:
        line_list = line.strip().split()
        timestamp = int(line_list[0])
        target_speed = float(line_list[3])
        current_speed = float(line_list[4])
        print_response(timestamp, target_speed, current_speed, MAX_SPEED, N_DOTS)
