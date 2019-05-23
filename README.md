This repository holds the miniproject for the "Fault detection, Isolation and Modelling" course,
group 831, Control and Automation Masters, 2nd Semester.

The miniproject aims to detect faults in a Welding system by using an Unknown input observer (UIO).
As the model of the welding system is unavailable, the Matlab System Identification toolbox is used to generate from system log files.

The fault.m files has initializes all the required parameters.
Run fault_solution.slx to simulate an UIO for the system.

fault_solution_mod is the same simulation where a sensor fault is added, and represents the final solution.
