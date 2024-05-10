#!/bin/bash

echo '
### TUSC EDIT ###

alias eb="gedit ~/.bashrc"
alias sb="source ~/.bashrc"

# additional routines for tusk
alias tusc_ready="cd ~/TUSC/BLDC_Test/pigpio-master && sudo ./pigpiod && cd .."
alias tusc_test="sudo python3 joystick_control_test.py"
alias tusc_calibrate="sudo python3 calibrate.py"
alias tusc="tusc_ready && tusc_test"

### END OF EDIT ###

' >> ~/.bashrc


