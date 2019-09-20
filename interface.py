import csv
import serial
import threading
import time

class iRobot(object):
  # Op Codes
	START = chr(128)
	READ_SENSORS = chr(148)
  
  # Packets
	WHEEL_DROP_AND_BUMPERS = chr(7) # 1 byte
	BUTTONS = chr(18) # 1 byte
	DISTANCE = chr(19) # 2 bytes
	ANGLE = chr(20) # 2 bytes
	BATTERY_CAPACITY = chr(26) # 2 bytes
	OI_MODE = chr(35) # 1 byte
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
		function that constantly updates the information from the sensors
		'''
		# Packets: 7, 18, 19, 20, 26, 35
		num_of_packets = len(iRobot.PACKETS)
		num_of_bytes = sum(iRobot.PACKETS.values())
		with open('data.csv') as data_file:
			data_writer = csv.writer(data_file, delimiter=',', quoting=csv.QUOTE_NONE)
			self.connection.write(iRobot.READ_SENSORS + chr(num_of_packets) + ''.join(iRobot.PACKETS.keys()))
			while (self.running):
				self.raw_data = self.connection.read(num_of_bytes)
				# TODO Check checksum
				# TODO Parse input
				# TODO Write to .csv
				data_writer.writerow([])
				time.sleep(0.020)
			data_file.close()
