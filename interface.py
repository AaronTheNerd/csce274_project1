import csv
import serial
import threading
import time
import math
from struct import pack, unpack
'''
Task 1. Augment the interface1 written for Task 2 in Project 1 including:
	a. Adding the possibility to set the robot to Full mode.
	b. The reading of the Bumps and Wheel Drops sensor data.
	c. The reading of all of the Cliff (packets 9-13, extremes included).
	d. The reading of the Angle and Distance (if not done in Project 1).
	e. The use of Drive Direct.
	f. Play a warning song. TODO
Task 2. Write a program that utilizes the augmented interface in the previous task and:
	a. Initializes the robot, by setting it in passive and safe mode (done in Project 1).
	b. If the robot is stopped, and none of the Wheel Drops and Cliff are activated, once the TODO
	clean/power button is pressed, it moves according to a random walk: the robot should
	move forward until it reaches an obstacle, then rotate in place for a 180 degrees plus a
	small random angle (between -45 and +45 degrees), then move forward again, and
	repeat. The rotation should be clockwise if the bumper left is pressed, while it should be
	counterclockwise if the bumper right is pressed. If both of them are pressed, take a
	random direction of rotation. Note that if the robot starts with a bumper pressed, it
	should rotate according to the rules described above.
	c. If the robot is moving, TODO
		i. when the clean/power button is pressed, stop the robot wherever it is.
		ii. check for the state of the Wheel Drops. In case any of them are activated, the
		robot should stop and play a warning song.
	d. Note that the program should not terminate and should continue listening for button TODO
	presses and check the state of the Wheel Drops and Cliff, both in b. and c. cases, until
	the program is terminated.
	e. Note also that when either bumper is pressed or any of the cliff sensors are triggered, TODO
	the robot should refuse to drive forward, but can still safely rotate in place. On the other
	hand, if any of the wheeldrop sensors are triggered, neither forward motion nor
	rotations are safe, because the robot is not resting properly on the ground.
	f. Once you are confident enough, you can activate the Full mode. TODO
	g. [Extra Credit] Log into a file the Distance and Angle, together with unsafe events
	(namely the wheel drops and/or cliffs) or button press, if happened, over time
	(timestamp should be included). The format of the file should be as follows:
	<timestamp>,<datum>
	where datum is distance, or angle, or the string UNSAFE, in case an unsafe event
	happened), or BUTTON, in case button pressed.
	h. [Extra Credit] Use threads to manage the motion of the robot and the reading of the TODO
	sensors. Remember that the connection is a shared resource among the different
	threads, as such you should use a way to synchronize the threads, e.g., Lock
	(https://docs.python.org/2/library/threading.html#lock-objects), as suggested in the
	Notes on the iRobot Create 2.
Task 3. [Extra Credit] Process the log file in order to plot the position of the robot in a 2D graph, TODO
e.g., using matplotlib (http://matplotlib.org/users/pyplot_tutorial.html). To plot the position of
the robot correctly, remember that the Create 2 is a differential drive robot and that the motion
you perform are forward motion and rotation in place. Note that this code should be separate
from the robot code and should be executed on your laptop by reading the log file created in
step (g).
'''

class Packet(object):
	'''
	A class that helps organize packets
	'''
	def __init__(self, _id, _bytes, _unpack):
		self.id = _id
		self.bytes = _bytes
		self.unpack = _unpack

class iRobot(object):
	'''
	A class that fully controls an iRobot
	'''

	################################################## Const Variables ##################################################

	# Op Codes
	RESET = chr(7)
	START = chr(128)
	SAFE = chr(131)
	FULL = chr(132)
	STOP = chr(173)
	DRIVE = 137
	DRIVE_DIRECT = 145
	READ_SENSORS = 148
  
	# Packets
	WHEEL_DROP_AND_BUMPERS = Packet(7, 1, 'BB')
	CLIFF_LEFT = Packet(9, 1, 'BB')
	CLIFF_FRONT_LEFT = Packet(10, 1, 'BB')
	CLIFF_FRONT_RIGHT = Packet(11, 1, 'BB')
	CLIFF_RIGHT = Packet(12, 1, 'BB')
	VIRTUAL_WALL = Packet(13, 1, 'BB')
	BUTTONS = Packet(18, 1, 'BB')
	DISTANCE = Packet(19, 2, 'Bh')
	ANGLE = Packet(20, 2, 'Bh')
	PACKETS = [WHEEL_DROP_AND_BUMPERS,
			   CLIFF_LEFT,
			   CLIFF_FRONT_LEFT,
			   CLIFF_FRONT_RIGHT,
			   CLIFF_RIGHT,
			   VIRTUAL_WALL,
			   BUTTONS,
			   DISTANCE,
			   ANGLE]

	SENSOR_READ_FORMAT = '>BB'
	for index in range(len(PACKETS)):
		SENSOR_READ_FORMAT += PACKETS[index].unpack
	SENSOR_READ_FORMAT += 'B'

	# Variables
	DELAY = 0.25 # s
	SENSOR_DELAY = 0.015 # s
	MAX_SPEED = 0.5 # m/s
	DIAMETER = 0.235 # m
	RADIUS = DIAMETER / 2.0 # m
	STRAIGHT = 32767
	CCW = 1
	CW = -1

	# States
	RUNNING = True
	MOVING = False

	def __init__(self):
		'''
		Establishes connection to the iRobot and creates a thread for data reading
		'''
		self.connection = serial.Serial('/dev/ttyUSB0', baudrate=115200) # Establish connection
		time.sleep(self.DELAY) # Wait
		self.data_thread = threading.Thread(target=self.read_data, args=(lambda : self.RUNNING,)) # Create a thread to read data
		self.RUNNING = True # Set a status boolean to True

		self.LWD = False
		self.RWD = False
		self.LB = False
		self.RB = False
		self.cliff_left = False
		self.cliff_front_left = False
		self.cliff_front_right = False
		self.cliff_right = False
		self.virtual_wall = False
		self.clock_pressed = False
		self.schedule_pressed = False
		self.day_pressed = False
		self.hour_pressed = False
		self.minute_pressed = False
		self.dock_pressed = False
		self.spot_pressed = False
		self.clean_pressed = False
		self.distance = 0
		self.angle = 0

	################################################## OI Mode and Starting ##################################################

	def start(self):
		'''
		Starts the iRobot and starts the data reading thread
		'''
		self.connection.write(self.START) # Send start command
		self.data_thread.start() # Start data thread
		self.start_time = time.time()
		time.sleep(self.DELAY) # Wait

	def reset(self):
		'''
		Resets the iRobot
		'''
		self.connection.write(self.RESET) # Send reset command
		time.sleep(self.DELAY) # Wait

	def stop(self):
		'''
		Stops the iRobot
		'''
		self.connection.write(self.STOP) # Send stop command
		self.connection.close()
		time.sleep(self.DELAY) # Wait
		self.RUNNING = False # Stop Data Thread
		self.data_thread.join()

	def safe(self):
		'''
		Sets the iRobot into safe mode
		'''
		self.connection.write(self.SAFE) # Send safe command
		time.sleep(self.DELAY) # Wait

	def full(self):
		'''
		Sets the iRobot into full mode
		'''
		self.connection.write(self.FULL) # Send full command
		time.sleep(self.DELAY) # Wait

	################################################## Sensor Reading ##################################################

	def read_data(self, running):
		'''
		Constantly updates the information from the sensors
		'''
		num_of_packets = len(self.PACKETS) # Define wanted number of packets
		num_of_bytes = num_of_packets # Define expected number of bytes with Header, n-bytes, and checksum
		for i in range(len(self.PACKETS)):
			num_of_bytes += self.PACKETS[i].bytes
		
		with open('data.csv', 'w+') as data_file: # Open file
			data_writer = csv.writer(data_file, delimiter=',', quoting=csv.QUOTE_NONE) # Open writer
			com = chr(self.READ_SENSORS) + chr(num_of_packets) # Pack
			for i in range(len(self.PACKETS)):
				com += chr(self.PACKETS[i].id)
			self.connection.write(com) # Send sensor command
			time.sleep(self.DELAY) # Wait
			while running: # Read data while running
				raw_data = self.connection.read(num_of_bytes + 3)
				raw_data = iRobot.unwrap(raw_data, chr(19), chr(num_of_bytes)) # Grab raw data and 'unwrap'
				data = unpack(self.SENSOR_READ_FORMAT, raw_data)[2:-1] # Unpack without header, n-bytes, and checksum
				self.parse_data(data) # Parse Data
				data_writer.writerow(['<' + str(time.time() - self.start_time) + '>', self.distance, self.angle, 'UNSAFE' if self.LB or self.RB else 'SAFE'])
				time.sleep(self.SENSOR_DELAY) # Wait
			data_file.close()

	@staticmethod
	def unwrap(raw_data, v1, v2):
		ret_list = raw_data
		for index in range(len(raw_data)):
			if raw_data[index] == v1 and raw_data[(index + 1) % len(raw_data)] == v2:
				ret_list = ''.join([raw_data[(i + index) % len(raw_data)] for i in range(len(raw_data))])
				break
		return ret_list

	def parse_data(self, data):
		'''
		Decodes all unpacked data
		'''
		for i in range(0, len(data), 2):
			if data[i] == self.WHEEL_DROP_AND_BUMPERS.id:
				self.decodeWDAB(data[i + 1])
			elif data[i] == self.CLIFF_LEFT.id:
				self.cliff_left = bool(data[i + 1])
			elif data[i] == self.CLIFF_FRONT_LEFT.id:
				self.cliff_front_left = bool(data[i + 1])
			elif data[i] == self.CLIFF_FRONT_RIGHT.id:
				self.cliff_front_right = bool(data[i + 1])
			elif data[i] == self.CLIFF_RIGHT.id:
				self.cliff_right = bool(data[i + 1])
			elif data[i] == self.VIRTUAL_WALL.id:
				self.virtual_wall = bool(data[i + 1])
			elif data[i] == self.BUTTONS.id:
				self.decodeB(data[i + 1])
			elif data[i] == self.DISTANCE.id:
				self.distance = data[i + 1]
			elif data[i] == self.ANGLE.id:
				self.angle = data[i + 1]
			else:
				print "Unknown ID found"
				break

	def decodeWDAB(self, data):
		'''
		Takes the byte that represents the wheel drop and bump sensors and decodes it
		'''
		self.LWD = bool(data & 8)
		self.RWD = bool(data & 4)
		self.LB = bool(data & 2)
		self.RB = bool(data & 1)

	def decodeB(self, data):
		'''
		Takes the byte that represents Buttons and decodes it
		'''
		self.clock_pressed = bool(data & 128)
		self.schedule_pressed = bool(data & 64)
		self.day_pressed = bool(data & 32)
		self.hour_pressed = bool(data & 16)
		self.minute_pressed = bool(data & 8)
		self.dock_pressed = bool(data & 4)
		self.spot_pressed = bool(data & 2)
		self.clean_pressed = bool(data & 1)

	################################################## Movement ##################################################

	def drive(self, speed=MAX_SPEED / 2.0, radius=STRAIGHT):
		'''
		Wrapper for drive commands, required from project 1
		'''
		self.connection.write(pack('>B2h', self.DRIVE, int(speed * 1000), radius))

	def drive_straight(self, distance, speed=MAX_SPEED / 2.0):
		'''
		Takes a distance in meters and a speed in meters per second and moves the iRobot the intended distance in a straight line
		'''
		speed = self.MAX_SPEED if speed > self.MAX_SPEED else speed
		t = distance / speed
		self.drive(speed)
		time.sleep(abs(t))
		self.stop_drive()

	def turn(self, angle, speed=MAX_SPEED / 5.0):
		'''
		Takes an angle in degrees and a speed in meters per second to rotate the iRobot in place
		If angle is positive then the iRobot will turn counter clockwise
		'''
		speed = self.MAX_SPEED if speed > self.MAX_SPEED else abs(speed)
		deg_per_sec = speed * 180.0 / (self.RADIUS * math.pi)
		t = angle / deg_per_sec
		if angle < 0:
			self.drive(speed, self.CW)
		else:
			self.drive(speed, self.CCW)
		time.sleep(abs(t))
		self.stop_drive()

	def stop_drive(self):
		'''
		Stops the iRobot's wheels
		'''
		self.drive(0, 0)
		time.sleep(self.DELAY)

	def drive_direct(self, t, vl, vr):
		'''
		Takes a time in seconds and 2 velocities in mm/s
		These represent the left and right wheel velocities
		'''
		self.connection.write(pack('>B2h', self.DRIVE_DIRECT, vl, vr)) # Send drive direct command
		time.sleep(t) # Wait for t seconds
		self.stop_drive() # Stop iRobot

	################################################## Magic Methods ##################################################
	def __str__(self):
		output = '<' + str(time.time() - self.start_time) + '>'
		output += '  Clean Button Pressed: ' + str(self.clean_pressed) + '\n'
		output += '  Distance: ' + str(self.distance) + '\n'
		output += '  Angle: ' + str(self.angle) + '\n'
		output += '  Left Bumper Pressed: ' + str(self.LB) + '\n'
		output += '  Right Bumper Pressed: ' + str(self.RB) + '\n'
		return output

	def __del__(self):
		self.stop()

################################################## Main Method ##################################################

if __name__ == "__main__":
	robot = iRobot()
	robot.start()
	robot.safe()
	while time.time() - robot.start_time < 50:
		print robot
		time.sleep(0.015)
	robot.stop()
