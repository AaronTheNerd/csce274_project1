import csv
import serial
import threading
import time
from collections import OrderedDict
from struct import pack

class iRobot(object):
	# Op Codes
	START = chr(128)
	SAFE = chr(131)
	DRIVE = chr(137)
	READ_SENSORS = 148
  
	# Packets
	WHEEL_DROP_AND_BUMPERS = 7 # 1 byte
	BUTTONS = 18 # 1 byte
	DISTANCE = 19 # 2 bytes
	ANGLE = 20 # 2 bytes
	BATTERY_CAPACITY = 26 # 2 bytes
	OI_MODE = 35 # 1 byte
	PACKETS = OrderedDict()
	PACKETS = {iRobot.WHEEL_DROP_AND_BUMPERS : 1,
			   iRobot.BUTTONS : 1,
			   iRobot.DISTANCE : 2,
			   iRobot.ANGLE : 2,
			   iRobot.BATTERY_CAPACITY : 2,
			   iRobot.OI_MODE : 1}

	def __init__(self):
		'''
		Establishes connection to the iRobot and creates a thread for data reading
		'''
		# Establish connection
		self.connection = serial.Serial('/dev/ttyUSB0', baudrate=115200)
		# Create a thread to read data
		self.data_thread = threading.Thread(target=self.read_data)
		# Set a status boolean to True
		self.running = True

	def start(self):
		'''
		Starts the iRobot and starts the data reading thread
		'''
		self.connection.write(iRobot.START)
		self.data_thread.start()

	def reset(self):
		'''
		Resets the iRobot
		'''
		return

	def stop(self):
		'''
		Stops the iRobot
		'''
		self.running = False

	def passive(self):
		'''
		Sets the iRobot into passive mode
		'''
		return

	def safe(self):
		'''
		Sets the iRobot into safe mode
		'''
		return

	def read_data(self):
		'''
		Constantly updates the information from the sensors
		'''
		num_of_packets = len(iRobot.PACKETS)
		num_of_bytes = sum(iRobot.PACKETS.values())
		with open('data.csv', 'w+') as data_file:
			data_writer = csv.writer(data_file, delimiter=',', quoting=csv.QUOTE_NONE)
			com = pack('>2h', iRobot.READ_SENSORS, num_of_packets)
			com += pack('>%sh' % num_of_packets, *iRobot.PACKETS.keys())
			self.connection.write(com)
			while self.running:
				self.raw_data = self.connection.read(num_of_bytes)
				checksum = 19 + self.raw_data[1] + self.raw_data[-1]
				fmt = '>2h'
				if checksum & 255 != 0:
					self.stop()
				data_writer.writerow([])
				time.sleep(0.020)
			data_file.close()
