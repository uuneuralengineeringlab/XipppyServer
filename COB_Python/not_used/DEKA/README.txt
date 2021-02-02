LowCostLkdriver README.txt

This contains important changes to the original lkdriver.h, lkdriver.cpp, TrainingSept8_1.py and TestingSept8_1.py (now lktraining.py and lktesting.py), and new setup.py and low_cost_lkdriver.py
that were required when wrapping it for use by the python based low cost control system.
In addition, this includes notes on each of the implementations and general notes on the project.

Aidan Lethaby
29 June 2020



low_cost_lkdriver.h:
  Notes/Changes
	-Changed '#pragma once' to more standard and universally supported '#ifndef' to prevent multiple inclusions

low_cost_lkdriver.cpp:
  Notes/Changes
	-Removed windows definitions for types such as DWORD, WORD, etc.
	-Wrapper for lk_get_sensor does not take in an array parameter, it recieves a Py_buffer and creates its own array from that to pass to the C++ function and 
	 returns a python double list
	-This is because the python program and C++ program do not share memory and cannot just pass the memory address between each other
	-We do not return the framecount to the python driver

low_cost_lkdriver.py:
  Notes
	-Provides python API for C++ based lkdriver
	-Provides manual terminal testing for hand
	-User can change how it deals with wrist control as it was originally for speed but can be made position controlled by use of written helper method
	-Demos in tester portion currently made to work with position control

lktraing.py/lktesting.py:
  Notes/Changes
        -Reformatted and eliminated unused code
	-Added entry point (ie. main method) to run when file is called in terminal
	-Replaced handi hand control with LUKE control
	-Rounded position values (mostly to help readability of csv file)

setup.py:
  Notes
	-Compiles low_cost_lkdriver.cpp into a python module to be used by low_cost_lkdriver.py
	-Useful to look at "Building C and C++ Extentions with distutils" at docs.python.org/3/extending/building.html



Code Relations:
  Diagram
					             _________________________________________       __________________________       __________________________   				
						    |                                         |     |                          |     |                          |
        input: low-cost control system EMG data --> |      lktraining.py or lktesting.py      | --> |   low_cost_lkdriver.py   | --> |  low_cost_lkdriver.cpp   | --> output: LUKE hand control
					            |_________________________________________|     |__________________________|     |__________________________|
  Notes
	-lktrainging/testing.py recieves EMG data from the low-cost control system and determines the six degrees of freedom in a standard -1 to 1 range and sends them to control the hand
	    -lktraining produces two .csv files containing the raw and averaged EMG data, then it produces and .pkl containing the trained data
	    -lktesting uses the .pkl data to run a kalman filter and control the hand
	-low_cost_lkdriver.py recieves standard six degrees of freedom and converts them to format the C++ driver can understand
	-low_cost_lkdriver.cpp establishes a connnection with the lk hand and sends commands over PCAN



General Notes and information:
  -All wrappers expect well formed types
  -Using API lk_set_command WRIST_ROTATION and WRIST_DEVIATION command values can be position if helper function is used (otherwise it is speed), lk_set_command_direct helper and original driver values for wrist represent the speed of movement
  -Commands using API:
      -Wrist rotation (- is counter-clockwise, + is clockwise, with palm up 0 is ~12 o'clock if position control)
      -Wrist deviation (- is inward, + is outward)
      -Thumb yaw (- is outward, + is inward)
      -Thumb pitch (- is inward, + is outward)
      -Index (- is outward, + is inward)
      -MRP (- is outward, + is inward)
  -See arduino_read_emg for file to get emg data from arduino mega and spiker shield
  -You can find PCAN drivers in home/pi
  -Useful to look at the "Buffer Protocol" and other documentation at docs.python.org
  -Useful to look at PCAN Driver for Linux v8 - User Manual for installation of drivers
  -Useful to refer to lkdriver folder used for MATLAB version of driver

Action Items:
  -Thumb pitch range seems much wider than MATLAB driver suggests, ask about this
  -Pinky sensor seems to be broken
  
