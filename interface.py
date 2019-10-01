import csv
import serial
import threading
import time
import math
from struct import pack, unpack

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
	PACKETS = {iRobot.WHEEL_DROP_AND_BUMPERS : 1,
			   iRobot.BUTTONS : 1,
			   iRobot.DISTANCE : 2,
			   iRobot.ANGLE : 2,
			   iRobot.BATTERY_CAPACITY : 2,
			   iRobot.OI_MODE : 1}
	PACKETS_FMT = {iRobot.WHEEL_DROP_AND_BUMPERS : 'BB',
				   iRobot.BUTTONS : 'BB',
				   iRobot.DISTANCE : 'Bh',
				   iRobot.ANGLE : 'Bh',
				   iRobot.BATTERY_CAPACITY : 'BH',
				   iRobot.OI_MODE : 'BB'}
	# Variables
	MAX_SPEED = 0.5 # m/s
	DIAMETER = 0.235 # m

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
		time.sleep(0.01)

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
		self.connection.write(iRobot.SAFE)
		time.sleep(0.01)

	def read_data(self):
		'''
		Constantly updates the information from the sensors
		'''
		num_of_packets = len(iRobot.PACKETS)
		num_of_bytes = sum(iRobot.PACKETS.values()) + num_of_packets + 3 # Header, n-bytes, checksum
		with open('data.csv', 'w+') as data_file:
			data_writer = csv.writer(data_file, delimiter=',', quoting=csv.QUOTE_NONE)
			com = pack('>2h', iRobot.READ_SENSORS, num_of_packets)
			com += pack('>%sh' % num_of_packets, *iRobot.PACKETS.keys())
			self.connection.write(com)
			while self.running:
				self.raw_data = self.connection.read(num_of_bytes)
				fmt = '>BB'
				for i in range(2, num_of_bytes - 1):
					if ord(self.raw_data[i]) in self.PACKETS_FMT.keys():
						i_fmt = self.PACKETS_FMT[ord(self.raw_data[i])]
						fmt += i_fmt
						i += self.PACKETS[ord(self.raw_data[i])]
					else:
						print "Unknown packet ID found"
						break
				fmt += 'B'
				self.data = unpack(fmt, self.raw_data)[2:-1]
				for i in range(0, len(self.data), step=2):
					if ord(self.data[i]) == iRobot.WHEEL_DROP_AND_BUMPERS:
						self.LWD = (self.data[i + 1] & 8) == 8
						self.RWD = (self.data[i + 1] & 4) == 4
						self.LB = (self.data[i + 1] & 2) == 2
						self.RB = (self.data[i + 1] & 1) == 1
					elif ord(self.data[i]) == iRobot.BUTTONS:
						self.clock_pressed = (self.data[i + 1] & 128) == 128
						self.schedule_pressed = (self.data[i + 1] & 64) == 64
						self.day_pressed = (self.data[i + 1] & 32) == 32
						self.hour_pressed = (self.data[i + 1] & 16) == 16
						self.minute_pressed = (self.data[i + 1] & 8) == 8
						self.dock_pressed = (self.data[i + 1] & 4) == 4
						self.spot_pressed = (self.data[i + 1] & 2) == 2
						self.clean_pressed = (self.data[i + 1] & 1) == 1
					elif ord(self.data[i]) == iRobot.DISTANCE:
						self.distance = self.data[i + 1]
					elif ord(self.data[i]) == iRobot.ANGLE:
						self.angle = self.data[i + 1]
					elif ord(self.data[i]) == iRobot.BATTERY_CAPACITY:
						self.battery_capacity = self.data[i + 1]
					elif ord(self.data[i]) == iRobot.OI_MODE:
						if self.data[i + 1] == 0:
							self.mode = 'OFF'
						elif self.data[i + 1] == 1:
							self.mode = 'PASSIVE'
						elif self.data[i + 1] == 2:
							self.mode = 'SAFE'
						else:
							self.mode = 'FULL'
				data_writer.writerow([self.LB, self.RB, self.clean_pressed, self.DISTANCE, self.angle])
				time.sleep(0.020)
			data_file.close()
			
	def drive(self, distance, speed=iRobot.MAX_SPEED):
		hex_speed = format(speed, '#06x')[2:]
		speed_high = '0x' + hex_speed[:2]
		speed_low = '0x' + hex_speed[2:]
		t = distance / speed
		self.connection.write(iRobot.DRIVE + chr(int(speed_high, 16)) + chr(int(speed_low, 16)) + chr(int('0x80', 16)) + chr(0))
		timer = threading.Timer(t, self.stop)
		timer.start()

	def turn(self, angle):
		deg_per_sec = iRobot.MAX_SPEED / (math.pi * iRobot.DIAMETER) * 360.0
		t = angle / deg_per_sec
		self.connection.write(iRobot.DRIVE + chr(0) + chr(0) + chr(int('0xFF', 16)) + chr(int('0xFF', 16)))
		timer = threading.Timer(t, self.stop)
		timer.start()

	def stop(self):
		self.connection.write(iRobot.DRIVE + chr(0) + chr(0) + chr(0) + chr(0))

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
