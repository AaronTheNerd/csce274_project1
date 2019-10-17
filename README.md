# CSCE 274 Robotics Project 2

### Task 1. Augment the interface1 written for Task 2 in Project 1 including:
	a. Adding the possibility to set the robot to Full mode.

	b. The reading of the Bumps and Wheel Drops sensor data.

	c. The reading of all of the Clif (packets 9-13, extremes included).
    
	d. The reading of the Angle and Distance (if not done in Project 1).
	
    e. The use of Drive Direct.
	
    f. Play a warning song.

### Task 2. Write a program that utilizes the augmented interface in the previous task and:
	a. Initializes the robot, by setting it in passive and safe mode (done in Project 1).
	
    b. If the robot is stopped, and none of the Wheel Drops and Cliff are activated, once the clean/power button is pressed, it moves according to a random walk: the robot should move forward until it reaches an obstacle, then rotate in place for a 180 degrees plus a small random angle (between -45 and +45 degrees),then move forward again, and repeat. The rotation should be clockwise if the bumper left is pressed, while it should be counterclockwise if the bumper right is pressed. If both of them are pressed, take a random direction of rotation. Note that if the robot starts with a bumper pressed, it should rotate according to the rules described above.
	
    c. If the robot is moving, 
		
        i. when the clean/power button is pressed, stop the robot wherever it is.
		
        ii. check for the state of the Wheel Drops. In case any of them are activated, the robot should stop and play a warning song.