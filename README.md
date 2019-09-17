# CSCE 274 Robotics Project 1

Task 1. Write an interface1
a. Connection to the serial interface.
b. Sending of commands.
c. Reading of data.
d. Close the connection.

Task 2. Using the interface in Task 1, write an interface for the robot that:
a. Control the state of the robot (Start, Reset, Stop, Passive, Safe).
b. Read the state of the buttons.
c. Send a Drive command to set the velocity and the radius of the wheels, given the two as
arguments.

Task 3. Write a program that utilizes the previous interfaces and:
a. Initializes the robot, by setting it in passive and safe mode.
b. If the robot is stopped, once the clean/power button is pressed, given an input N, move
counterclockwise along a regular polygon with N sides and total perimeter of 2m
(meters). The robot stops once the polygon is covered. Your solution should work for
any valid value of N.
c. If the robot is moving, when the clean/power button is pressed, stop the robot when it
reaches the current goal vertex.