import csv
import serial
import threading
import time
import math
from struct import pack, unpack

class iRobot(object):
	# Op Codes
	RESET = chr(7)
	START = chr(128)
	SAFE = chr(131)
	STOP = chr(173)
	DRIVE = 137
	READ_SENSORS = 148
  
	# Packets
	WHEEL_DROP_AND_BUMPERS = 7 # 1 byte
	BUTTONS = 18 # 1 byte
	DISTANCE = 19 # 2 bytes
	ANGLE = 20 # 2 bytes
	BATTERY_CAPACITY = 26 # 2 bytes
	OI_MODE = 35 # 1 byte
	PACKETS = {BUTTONS : 1}
	PACKETS_FMT = {BUTTONS : 'BB'}

	# Variables
	DELAY = 0.1 # s
	MAX_SPEED = 0.5 # m/s
	DIAMETER = 0.235 # m

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

	def stop(self):
		'''
		Stops the iRobot
		'''
		# Stop running
		self.running = False
		# Send stop command
		self.connection.write(self.STOP)
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

	def read_data(self):
		'''
		Constantly updates the information from the sensors
		'''
		# Define wanted number of packets
		num_of_packets = len(self.PACKETS)
		# Define expected number of bytes
		num_of_bytes = sum(self.PACKETS.values()) + num_of_packets + 3 # Header, n-bytes, checksum
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
			time.sleep(self.DELAY)
			# Read data while running
			while self.running:
				# Grab raw data without header, n-bytes, and checksum
				self.raw_data = self.connection.read(num_of_bytes)[2:-1]
				# Build Format
				fmt = self.build_fmt(self.raw_data)
				# Unpack return
				self.data = unpack(fmt, self.raw_data)
				# Parse Data
				self.decode(self.data)
				# Write sensor output
				data_writer.writerow([self.LB, self.RB, self.clean_pressed, self.DISTANCE, self.angle])
				# Wait
				time.sleep(self.DELAY)
			data_file.close()

	def build_fmt(self, raw_data):
		fmt = ''
		for i in range(len(raw_data)):
			try:
				i_fmt = self.PACKETS_FMT[ord(raw_data[i])]
				fmt += i_fmt
				i += self.PACKETS[ord(raw_data[i])]
			except:
				print "Unknown packet ID found"
				break
		return fmt

	def decode(self, data):
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
		self.LWD = (data & 8) == 8
		self.RWD = (data & 4) == 4
		self.LB = (data & 2) == 2
		self.RB = (data & 1) == 1

	def decodeB(self, data):
		self.clock_pressed = (data & 128) == 128
		self.schedule_pressed = (data & 64) == 64
		self.day_pressed = (data & 32) == 32
		self.hour_pressed = (data & 16) == 16
		self.minute_pressed = (data & 8) == 8
		self.dock_pressed = (data & 4) == 4
		self.spot_pressed = (data & 2) == 2
		self.clean_pressed = (data & 1) == 1

	def decodeOI(self, data):
		if data == 0:
			self.mode = 'OFF'
		elif data == 1:
			self.mode = 'PASSIVE'
		elif data == 2:
			self.mode = 'SAFE'
		else:
			self.mode = 'FULL'

	def drive(self, distance, speed=MAX_SPEED):
		hex_speed = format(int(speed), '#06x')[2:]
		speed_high = '0x' + hex_speed[:2]
		speed_low = '0x' + hex_speed[2:]
		t = distance / speed
		self.connection.write(pack('>B2h', self.DRIVE, speed * 1000, 0))
		time.sleep(t)
		self.stop()

	def turn(self, angle, speed=MAX_SPEED):
		deg_per_sec = speed * 360.0 / (math.pi * self.DIAMETER)
		t = angle / deg_per_sec
		self.connection.write(pack('>B2h', self.DRIVE, speed * 1000, 1))
		time.sleep(t)
		self.stop()

	def stop(self):
		self.connection.write(pack('>B2h', self.DRIVE, 0, 0))

def main():
	robot = iRobot()
	robot.start()
	robot.safe()
	N = 3 # Num of sides
	L = 2.0 / N # Length of a side
	D = ((N - 2) * 180.0) / N # Interior Angle
	for i in range(N):
		robot.drive(L)
		robot.turn(D)

if __name__ == "__main__":
	main()
