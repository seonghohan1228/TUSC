#!/bin/bash

echo '
### TUSC EDIT ###

alias eb="gedit ~/.bashrc"
alias sb="source ~/.bashrc"

# additional routines for tusk
alias tusc_ready="cd ~/TUSC/pigpio-master && sudo ./pigpiod && cd .."
alias tusc_run="sudo python3 ~/TUSC/run.py"
alias tusc_calibrate="sudo python3 ~/TUSC/_legacy/calibrate.py"
alias tusc="tusc_ready && tusc_run"

### END OF EDIT ###

' >> ~/.bashrc


