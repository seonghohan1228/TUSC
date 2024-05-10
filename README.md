# ME400 Team 1 Group B TUSC (Tracks Used for Stair Climbing)

# Initializing Environment
Edit `.bashrc` by running the bash script `edit_bash`:
```console
user@pi:~$ ./edit_bash.sh
```

This will add relevant aliases to the `.bashrc` file.


# Installing and running pigpiod
To install the Raspberry Pi's GPIO utility: pigpiod, execute the setup script:
```console
user@pi:~$ ./pigpiod_setup.sh
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

