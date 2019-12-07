import serial
import threading
import time
import math
from struct import pack, unpack

class Connection(object):
	'''
	Wrapper class for serial connection
	'''
	DELAY = 0.015
	def __init__(self):
		'''
		Establish connection immdeiately upon being called
		'''
		self.connection = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)
		self.lock = threading.Lock()
		time.sleep(self.DELAY)
  
	def send(self, data, delay=True):
		'''
		This takes an input string data, and sends it to the iRobot
		'''
		self.lock.acquire()
		try:
			self.connection.write(data)
		finally:
			if delay:
				time.sleep(self.DELAY)
			self.lock.release()
  
	def receive(self, n):
		'''
		This sends the command to read n number of bytes from iRobot
		'''
		return self.connection.read(n)

	def close(self):
		'''
		Closes connection to iRobot
		'''
		self.connection.close()

class Packet(object):
	'''
	A class that helps organize packets
	'''
	def __init__(self, _id, _bytes, _unpack):
		self.id = _id
		self.bytes = _bytes
		self.unpack = _unpack

class Button(object):
	'''
	A class to make better button responses
	'''
	def __init__(self):
		'''
		Creates a button
		'''
		self.pressed = False
		self.released = False

	def update_button(self, new_state):
		'''
		Takes a new state and finds if the button is being pressed or has been released
		'''
		self.released = True if self.pressed == True and new_state == False else False
		self.pressed = new_state

	def reset(self):
		'''
		Resets the button
		'''
		self.pressed = False
		self.released = False

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
	SEEK_DOCK = 143
  
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
	LIGHT_BUMPERS = Packet(45, 1, 'BB')
	LIGHT_BUMP_RIGHT = Packet(51, 2, 'BH')
	IR_LEFT = Packet(52, 1, 'BB')
	IR_RIGHT = Packet(53, 1, 'BB')
	IR_OMNI = Packet(17, 1, 'BB')
	PACKETS = [IR_LEFT, IR_RIGHT, IR_OMNI, BUTTONS, LIGHT_BUMP_RIGHT, WHEEL_DROP_AND_BUMPERS, LIGHT_BUMPERS]

	SENSOR_READ_FORMAT = '>BB'
	for index in range(len(PACKETS)):
		SENSOR_READ_FORMAT += PACKETS[index].unpack
	SENSOR_READ_FORMAT += 'B'

	# Variables
	SENSOR_DELAY = 0.020 # s
	MAX_SPEED = 0.5 # m/s
	DIAMETER = 0.235 # m
	RADIUS = DIAMETER / 2.0 # m
	STRAIGHT = 32767
	CCW = 1
	CW = -1
	BUTTON_INTERRUPT = NameError('Button Pressed')

	# Infrared Characters
	RED_BUOY = 168
	GREEN_BUOY = 164
	FORCE_FIELD = 161
	G_N_R_BUOY = 172
	G_N_FF = 165
	R_N_FF = 169
	R_N_G_N_FF = 173

	def __init__(self):
		'''
		Establishes connection to the iRobot and creates a thread for data reading
		'''
		self.connection = Connection() # Establish connection
		self.data_thread = threading.Thread(target=self.read_data) # Create a thread to read data
		self.data_thread.daemon = True

		self.LWD = False # Left wheel drop
		self.RWD = False # Right wheel drop
		self.LB = False # Left bumper
		self.RB = False # Right bumper
		self.cliff_left = False
		self.cliff_front_left = False
		self.cliff_front_right = False
		self.cliff_right = False
		self.virtual_wall = False
		self.clock = Button()
		self.schedule = Button()
		self.day = Button()
		self.hour = Button()
		self.minute = Button()
		self.dock = Button()
		self.spot = Button()
		self.clean = Button()
		self.distance = 0
		self.angle = 0
		self.IR_BR = 0 # Infrared right sensor
		self.IR_BL = 0 # Infrared left sensor
		self.LT_BR = False # Light bump right
		self.LT_BFR = False # Light bump front right
		self.LT_BCR = False # Light bump center right
		self.LT_BCL = False # Light bump center left
		self.LT_BFL = False # Light bump front left
		self.LT_BL = False # Light bump left
		self.IR_LEFT_CHAR = 0 # Infrared's left character
		self.IR_RIGHT_CHAR = 0 # Infrared's right character
		self.IR_OMNI_CHAR = 0 # Infrared's omni character

	################################################## OI Mode and Starting ##################################################

	def start(self):
		'''
		Starts the iRobot and starts the data reading thread
		'''
		self.connection.send(self.START) # Send start command
		self.data_thread.start() # Start data thread
		self.start_time = time.time()

	def reset(self):
		'''
		Resets the iRobot
		'''
		self.connection.send(self.RESET) # Send reset command

	def stop(self):
		'''
		Stops the iRobot
		'''
		self.connection.send(self.STOP) # Send reset command

	def safe(self):
		'''
		Sets the iRobot into safe mode
		'''
		self.connection.send(self.SAFE) # Send reset command

	def full(self):
		'''
		Sets the iRobot into full mode
		'''
		self.connection.send(self.FULL) # Send reset command

	################################################## Sensor Reading ##################################################

	def read_data(self):
		'''
		Constantly updates the information from the sensors
		'''
		num_of_packets = len(self.PACKETS) # Define wanted number of packets
		num_of_bytes = num_of_packets # Define expected number of bytes without Header, n-bytes, and checksum
		for i in range(len(self.PACKETS)):
			num_of_bytes += self.PACKETS[i].bytes
		com = chr(self.READ_SENSORS) + chr(num_of_packets) # Create command
		for i in range(len(self.PACKETS)):
			com += chr(self.PACKETS[i].id)
		self.connection.send(com) # Send sensor command
		while True: # Read data while running
			raw_data = self.connection.receive(num_of_bytes + 3)
			raw_data = iRobot.unwrap(raw_data, chr(19), chr(num_of_bytes)) # Grab raw data and 'unwrap'
			if len(raw_data) < (2 * num_of_packets):
				continue
			data = unpack(self.SENSOR_READ_FORMAT, raw_data)[2:-1] # Unpack without header, n-bytes, and checksum
			self.parse_data(data) # Parse Data
			global flagStop
			if self.clean.released:
				if flagStop == True:
					flagStop = False
				else:
					flagStop = True
			#print(flagStop)

		
		
	@staticmethod
	def unwrap(raw_data, v1, v2):
		'''
		Assumes that a list is circular and forces v1 and v2 to be the first 2 values
		'''
		ret_list = []
		for index in range(len(raw_data)):
			if raw_data[index] == v1 and raw_data[(index + 1) % len(raw_data)] == v2:
				ret_list = ''.join([raw_data[(i + index) % len(raw_data)] for i in range(len(raw_data))])
				break
		return ret_list

	def parse_data(self, data):
		'''
		Decodes all unpacked data
		'''
		if data == []:
			return
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
				self.distance += data[i + 1]
			elif data[i] == self.ANGLE.id:
				self.angle += data[i + 1]
				self.angle %= 360
			elif data[i] == self.LIGHT_BUMP_RIGHT.id:
				self.IR_BR = data[i + 1]
			elif data[i] == self.LIGHT_BUMPERS.id:
				self.decodeLTBS(data[i + 1])
			elif data[i] == self.IR_LEFT.id:
				self.IR_LEFT_CHAR = int(data[i + 1])
			elif data[i] == self.IR_RIGHT.id:
				self.IR_RIGHT_CHAR = int(data[i + 1])
			elif data[i] == self.IR_OMNI.id:
				self.IR_OMNI_CHAR = int(data[i + 1])
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
		self.clock.update_button(bool(data & 128))
		self.schedule.update_button(bool(data & 64))
		self.day.update_button(bool(data & 32))
		self.hour.update_button(bool(data & 16))
		self.minute.update_button(bool(data & 8))
		self.dock.update_button(bool(data & 4))
		self.spot.update_button(bool(data & 2))
		self.clean.update_button(bool(data & 1))

	def decodeLTBS(self, data):
		'''
		'''
		self.LT_BR = bool(data & 32)
		self.LT_BFR = bool(data & 16)
		self.LT_BCR = bool(data & 8)
		self.LT_BCL = bool(data & 4)
		self.LT_BFL = bool(data & 2)
		self.LT_BL = bool(data & 1)


	################################################## Movement ##################################################

	def drive(self, speed=MAX_SPEED / 5.0, radius=STRAIGHT, delay=False):
		'''
		Wrapper for drive commands, required from project 1
		'''
		self.connection.send(pack('>B2h', self.DRIVE, int(speed * 1000), radius), delay=delay)
		#if self.clean.pressed:
			#raise self.BUTTON_INTERRUPT

	def drive_straight(self, distance, speed=MAX_SPEED / 5.0):
		'''
		Takes a distance in meters and a speed in meters per second and moves the iRobot the intended distance in a straight line
		'''
		speed = self.MAX_SPEED if speed > self.MAX_SPEED else -self.MAX_SPEED if speed < -self.MAX_SPEED else speed # Makes sure that the speed isn't too high or too low
		t = distance / speed # Solve for t
		drive_start_time = time.time() # Keep track of start time
		self.drive(speed) # Send drive command
		while (time.time() - drive_start_time < t and self.safe_to_drive() and not self.clean.pressed): # Make sure the iRobot is safe to drive and hasn't driven too far
			continue
		self.stop_drive() # Send stop driving command
		if self.clean.pressed:
			raise self.BUTTON_INTERRUPT

	def turn(self, angle, speed=MAX_SPEED / 5.0):
		'''
		Takes an angle in degrees and a speed in meters per second to rotate the iRobot in place
		If angle is positive then the iRobot will turn counter clockwise
		'''
		speed = self.MAX_SPEED if speed > self.MAX_SPEED else -self.MAX_SPEED if speed < -self.MAX_SPEED else speed # Makes sure that the speed isnt too fast
		deg_per_sec = speed * 180.0 / (self.RADIUS * math.pi) # Find degrees per second
		t = angle / deg_per_sec # Solve for t
		if angle < 0: # Figure out which way to rotate
			self.drive(speed, self.CW)
		else:
			self.drive(speed, self.CCW)
		drive_start_time = time.time() # Keep track of start time
		while (time.time() - drive_start_time < abs(t) and self.safe_to_turn() and not self.clean.pressed): # Make sure the iRobot is safe to turn and hasn't turned for too long
			continue
		self.stop_drive() # Stop turning
		if self.clean.pressed:
			raise self.BUTTON_INTERRUPT

	def stop_drive(self):
		'''
		Stops the iRobot's wheels
		'''
		self.drive(0, 0, delay=True) # Sends stop command with delay

	def drive_direct(self, t, vl, vr):
		'''
		Takes a time in seconds and 2 velocities in mm/s
		These represent the left and right wheel velocities
		'''
		self.connection.send(pack('>B2h', self.DRIVE_DIRECT, vl, vr), delay=False) # Send drive direct command
		drive_start_time = time.time() # Keep track of when the iRobot started driving
		while ((time.time() - drive_start_time < t) and self.safe_to_drive()): # While can drive and hasn't driven for 't' seconds
			continue
		self.stop_drive() # Stop iRobot

	################################################## Magic Methods ##################################################
	def __str__(self):
		'''
		Creates a string that shows the iRobot's current runtime, and other data
		'''
		output = '<' + str(time.time() - self.start_time) + '>'
		output += '  Clean Button Pressed: ' + str(self.clean.pressed) + '\n'
		output += '  Distance: ' + str(self.distance) + '\n'
		output += '  Angle: ' + str(self.angle) + '\n'
		output += '  Left Bumper Pressed: ' + str(self.LB) + '\n'
		output += '  Right Bumper Pressed: ' + str(self.RB) + '\n'
		return output

	def safe_to_drive(self):
		'''
		Returns a boolean that represents whether or not the iRobot is safe to drive
		'''
		return not (self.LWD or self.RWD or self.LB or self.RB or self.cliff_front_left or self.cliff_front_right or self.cliff_left or self.cliff_right)

	def safe_to_turn(self):
		'''
		Returns a boolean that represents whether or not the iRobot is safe to rotate
		'''
		return not (self.LWD or self.RWD)

	def button_pressed(self):
		'''
		Returns a boolean that represents whether or not the iRobot has had a button pressed
		'''
		return self.clean.pressed or self.dock.pressed or self.spot.pressed or self.schedule.pressed or self.clock.pressed or self.day.pressed or self.hour.pressed or self.minute.pressed

	def play_song(self):
		'''
		Creates and plays a song
		'''
		song = chr(59) + chr(16) + chr(55) + chr(16) + chr(60) + chr(16) + chr(55) + chr(16) + chr(62) + chr(16) + chr(55) + chr(16) + chr(63) + chr(16) + chr(55) + chr(16) + chr(62) + chr(16) + chr(55) + chr(16) + chr(60) + chr(16) + chr(55) + chr(16) + chr(40) + chr(8) + chr(41) + chr(8) + chr(42) + chr(8) + chr(43) + chr(16)
		self.connection.send(chr(140) + chr(0) + chr(16) + song)
		self.connection.send(chr(141) + chr(0))

	@staticmethod
	def error2radius(error):
		'''
		Maps an error from a controller to an appropriate radius for wall following
		'''
		if error == 0:
			return iRobot.STRAIGHT
		elif error < 0:
			return int(iRobot.bound(-iRobot.STRAIGHT / error + 500, -iRobot.STRAIGHT + 1, iRobot.STRAIGHT))
		else:
			return int(iRobot.bound(-iRobot.STRAIGHT / error + 100, -iRobot.STRAIGHT + 1, iRobot.STRAIGHT))

	@staticmethod
	def bound(val, min_, max_):
		'''
		Bounds a value between a lower and upper bound
		'''
		if val < min_:
			return min_
		elif val > max_:
			return max_
		return val

################################################## Main Method ##################################################

if __name__ == "__main__":
	global flagStop
	flagStop = True
	Kp = 0.3 # Arbitrary proprtional gain
	Kd = 0.0075 # Arbitrary derivative gain
	set_point = 420 # Arbitrary set point
	delay = 0.25
	error = lambda e_prev_, e_curr_: (Kp * e_curr_) + (Kd * (e_curr_ - e_prev_) / delay)
	robot = iRobot()
	robot.start()
	robot.safe()
	time.sleep(0.1)
	e_prev = set_point - robot.IR_BR # Previous error
	e_curr = e_prev # Current error
	while True:
			if robot.hour.pressed: # Dev full stop
				break
			if robot.clean.released: # Start moving once clean is pressed
				while True:
					print "Omni:", robot.IR_OMNI_CHAR
					if robot.IR_OMNI_CHAR == iRobot.GREEN_BUOY:
						print "  Green Buoy"
					elif robot.IR_OMNI_CHAR == iRobot.RED_BUOY:
						print "  Red Buoy"
					elif robot.IR_OMNI_CHAR == iRobot.FORCE_FIELD:
						print "  Force Field"
					elif robot.IR_OMNI_CHAR == iRobot.G_N_R_BUOY:
						print "  Green and Red Buoy"
					elif robot.IR_OMNI_CHAR == iRobot.G_N_FF:
						print "  Green and Force Field"
					elif robot.IR_OMNI_CHAR == iRobot.R_N_FF:
						print "  Red and Force Field"
					elif robot.IR_OMNI_CHAR == iRobot.R_N_G_N_FF:
						print "  All IR Chars"
					else:
						print "  None"
					print "####################"
					'''
					Seek Dock:
					If only omni char, rotate until left or right char
					Use left and right chars to direct roomba to dock
					Once left and right receive green and red, drive straight
					Once Center bump, stop driving
					'''
					try:
						if flagStop == True: # Stop robot when button pressed
							robot.stop_drive()
							break
						if flagStop == False: # Wall follow while no button is pressed
							e_prev, e_curr = e_curr, set_point - robot.IR_BR # Set error
							e_val = error(e_prev, e_curr) # Find an error value
							radius = iRobot.error2radius(e_val) # Find a required radius
							robot.drive(iRobot.MAX_SPEED / 4.5, radius) # Drive while safe
							if robot.clean.released:
								break
							if robot.LT_BFR or (robot.RB and not robot.LB): # If the robot is pointed towards the wall, turn left
								robot.drive(iRobot.MAX_SPEED / 4.5, iRobot.CCW)
								while robot.LT_BFR or (robot.RB and not robot.LB):
									continue
								robot.stop_drive()
							if (robot.LB and robot.RB) or robot.LT_BCL: # If the robot has a center bump or sees a wall infront of it, turn left
								robot.drive(iRobot.MAX_SPEED / 4.5, iRobot.CCW)
								while (robot.LB and robot.RB) or robot.LT_BCL:
									continue
								robot.stop_drive()
							time.sleep(delay)
					except:
						break
	robot.stop_drive()
	robot.stop()