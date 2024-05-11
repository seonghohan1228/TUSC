# ME400 Team 1 Group B TUSC (Tracks Used for Stair Climbing)

# Initializing Environment

## Editing `.bashrc`
Edit `.bashrc` by typing:
```console
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

source ~/.bashrc
```

This will add relevant aliases to the `.bashrc` file.


## Installing and running pigpiod
To install the Raspberry Pi's GPIO utility: pigpiod, type:
```console
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
cd ..
```

# Running TUSC
When running TUSC, run the following commands.

If TUSC has been booted for the **first** time:
```console
user@pi:~$ tusc
```

If `tusc` or `tusc_ready` has been run, or gpiod has been already started:
```console
user@pi:~$ tusc_run
```

