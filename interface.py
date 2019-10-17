import csv
import serial
import threading
import time
import math
from struct import pack, unpack
'''
Task 1. Augment the interface1 written for Task 2 in Project 1 including:
	a. Adding the possibility to set the robot to Full mode.
	b. The reading of the Bumps and Wheel Drops sensor data. TODO
	c. The reading of all of the Cliff (packets 9-13, extremes included). TODO
	d. The reading of the Angle and Distance (if not done in Project 1). TODO
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
'''

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
	WHEEL_DROP_AND_BUMPERS = 7 # 1 byte
	BUTTONS = 18 # 1 byte
	DISTANCE = 19 # 2 bytes
	ANGLE = 20 # 2 bytes
	PACKETS = {BUTTONS : 1}
	PACKETS_FMT = {BUTTONS : 'BB'}

	# Variables
	DELAY = 0.25 # s
	SENSOR_DELAY = 0.015 # s
	MAX_SPEED = 0.5 # m/s
	DIAMETER = 0.235 # m
	RADIUS = DIAMETER / 2.0 # m
	STRAIGHT = 32767
	CCW = 1
	CW = -1

	def __init__(self):
		'''
		Establishes connection to the iRobot and creates a thread for data reading
		'''
		# Establish connection
		self.connection = serial.Serial('/dev/ttyUSB0', baudrate=115200)
		# Wait
		time.sleep(self.DELAY)
		# Create a thread to read data
		self.data_thread = threading.Thread(target=self.read_data)
		# Set a status boolean to True
		self.running = True

	################################################## OI Mode and Starting ##################################################

	def start(self):
		'''
		Starts the iRobot and starts the data reading thread
		'''
		# Send start command
		self.connection.write(self.START)
		# Start data thread
		self.data_thread.start()
		# Wait
		time.sleep(self.DELAY)

	def reset(self):
		'''
		Resets the iRobot
		'''
		# Send reset command
		self.connection.write(self.RESET)
		# Wait
		time.sleep(self.DELAY)

	def stop(self):
		'''
		Stops the iRobot
		'''
		# Stop running
		# Send stop command
		self.connection.write(self.STOP)
		self.connection.close()
		# Wait
		time.sleep(self.DELAY)

	def safe(self):
		'''
		Sets the iRobot into safe mode
		'''
		# Send safe command
		self.connection.write(self.SAFE)
		# Wait
		time.sleep(self.DELAY)

	def full(self):
		'''
		Sets the iRobot into full mode
		'''
		# Send full command
		self.connection.write(self.FULL)
		# Wait
		time.sleep(self.DELAY)

	################################################## Sensor Reading ##################################################

	def read_data(self):
		'''
		Constantly updates the information from the sensors
		'''
		# Define wanted number of packets
		num_of_packets = len(self.PACKETS)
		# Define expected number of bytes
		num_of_bytes = sum(self.PACKETS.values()) + len(self.PACKETS) + 3 # Header, n-bytes, checksum
		# Open file
		with open('data.csv', 'w+') as data_file:
			# Open writer
			data_writer = csv.writer(data_file, delimiter=',', quoting=csv.QUOTE_NONE)
			# Pack
			com = pack('>2B', self.READ_SENSORS, num_of_packets)
			com += pack('>%sB' % num_of_packets, *self.PACKETS.keys())
			# Send command
			self.connection.write(com)
			# Wait
			time.sleep(self.SENSOR_DELAY)
			# Read data while running
			while self.running:
				# Grab raw data without header, n-bytes, and checksum
				self.raw_data = self.connection.read(size=num_of_bytes)
				print "Raw Data:", self.raw_data
				# Build Format
				fmt = '>BB'
				# Unpack return
				self.data = unpack(fmt, self.raw_data)
				print self.data
				# Parse Data
				self.decodeB(self.data[1])
				# Write sensor output
				print [self.clean_pressed, self.clock_pressed, self.day_pressed]
				# Wait
				time.sleep(self.SENSOR_DELAY)
			data_file.close()

	def build_fmt(self, raw_data):
		'''
		Dynamically develops a format to unpack the input
		'''
		fmt = '>'
		for i in range(len(raw_data)):
			try:
				fmt += self.PACKETS_FMT[ord(raw_data[i])]
				i += self.PACKETS[ord(raw_data[i])]
			except:
				print "Unknown packet ID found"
				break
		return fmt

	def decode(self, data):
		'''
		Decodes all unpacked data
		'''
		for i in range(0, len(data), step=2):
			if ord(data[i]) == self.WHEEL_DROP_AND_BUMPERS:
				self.decodeWDAB(data[i + 1])
			elif ord(data[i]) == self.BUTTONS:
				self.decodeB(data[i + 1])
			elif ord(data[i]) == self.DISTANCE:
				self.distance = data[i + 1]
			elif ord(data[i]) == self.ANGLE:
				self.angle = data[i + 1]
			elif ord(data[i]) == self.BATTERY_CAPACITY:
				self.battery_capacity = data[i + 1]
			elif ord(data[i]) == self.OI_MODE:
				self.decodeOI(data[i + 1])
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
		# Send drive direct command
		self.connection.write(pack('>B2h', self.DRIVE_DIRECT, vl, vr))
		# Wait for t seconds
		time.sleep(t)
		# Stop iRobot
		self.stop_drive()

################################################## Main Method ##################################################

if __name__ == "__main__":
	robot = iRobot()
	robot.start()
	robot.safe()
	N = 4 # Num of sides
	L = 2.0 / N # Length of a side
	D = 360.0 / N # Angle per corner
	for i in range(N):
		robot.drive_straight(L)
	 	robot.turn(D)
		time.sleep(0.010)
	time.sleep(5)
	robot.stop()
