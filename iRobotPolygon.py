# CSCE 274 iRobot Project #1
# CC 2019 Nicholas Mueller, Tyler Morehead, Aaron Barge

import serial
import time
import struct
import sys

#
# ==== CONSTANTS ====
#
PAUSE = 0.5 # Constant for min time to pause command execution, to prevent two commands from sending at once
WAIT = 10 # Constant time value (seconds) for iRobot to wait after initialization before executing movement
TOTAL_TURN = 6.6 # Constant value as baseline for how long, in seconds, it takes iRobot to turn 2pi rad at 100mm/s
TOTAL_FWD = 20.0 # Constant value as baseline for how long, in seconds, it takes iRobot to drive for 2m at 100mm/s
WHEEL_VEL = 100 # Constant speed for wheel velocity, in mm/s
WHEEL_RAD_S = 0 # Constant value for moving in straight line, i.e no turning radius at all
WHEEL_RAD_T = 1 # Constant value for turning at a radius of 1mm
                      # We want such a small turning radius so that iRobot turns on the spot without moving forward or backward
#
# ==== CONSTANTS ====
#

#
#
# ==== CLASSES/INTERFACES ====
#
#

#Connector Class
# This controls basic requirements for connecting to the iRobot, specifically establishing the connection, sending commands and reading from the iRobot
class PiConnector:
  def __init__(self):
    self.connection = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)
      # Establish connection immediately upon being called
  
  def send(self, data):
    self.connection.write(data)
      # This takes an input string data, and sends it to the iRobot
  
  def receive(self, n):
    self.connection.read(n)
      # This sends the command to read n number of bytes from iRobot

  def close(self):
    self.connection.close()
      # Closes connection to iRobot

class roombControl:
  # This class controls the managing of commands, and uses PiConnector to send and receive them from the iRobot
  def __init__(self):
    self.connection = PiConnector()
  
  def setStart(self):
    self.connection.send(chr(128))
      #Starts the Open Interface of iRobot, allowing other commands to be sent
  def setSafe(self):
    self.connection.send(chr(131))
      # Sets the iRobot to safe mode, where all features are allowed to be accessed and changed
  def setStop(self):
    self.connection.send(chr(173))
    sleep(PAUSE)
    self.connection.close()
      # Terminates the Open Interface, and closes connection to iRobot
  def reset(self):
    self.connection.send(chr(7))
      # completely resets iRobot. Again, must be set to start again before commands can be sent

  def buttonState(self):
    # Sends request to iRobot for sensor data on buttons, then reads and interprets the returned data
    
    '''
      This would typically include a command such as
      self.connection.write(chr(142)+chr(18)) to send request for sensor data about button state (packet ID 18)
      followed by a receive(n) since it will receive one byte of data, which will then be interpreted to know the 
      state of whichever button is needed. However, in testing I was unable to properly read the correct data, as
      the interpreted data consistently only returned a sequence of seemingly random numbers, which translated to
      an apparent state of random buttons constantly being pressed, regardless of whether they actually were
    '''

  def drive(self, vel, rad, sec):
    # This command takes a given velocity(mm/s) and wheel radius, and sends the proper command to the iRobot to then drive at that velocity for a given amount of seconds.
    data = struct.pack('>B2h', 137, vel, rad)
    stopData = struct.pack('>B2h', 137, 0, rad)
      # The command structure for the drive command uses the format [opcode][vel H-Byte][vel L-Byte][rad H-Byte][rad L-Byte].
      # In order to avoid manually computing the 2's compliment hex value to turn an integer into two split hex values, pack with >B2h formatting is used to do it automatically
    self.connection.send(data)
    time.sleep(sec)
      # Used to allow iRobot to drive for sec seconds before stopping
    self.connection.send(stopData)
    time.sleep(PAUSE) # Ensure any command sent directly after moving goes through
#
#
# ==== CLASSES/INTERFACES ====
#
#

#
#
#
# ==== MAIN PROGRAM ====
#
#
#
'''
  This program will, using prior classes, establish connection with iRobot,
  and given a main argument of N (called upon initialization of the program sys.argv[1])
  will proceed to trace a polygon of N sides on the floor. The polygon will
  have a total perimeter of 2 meters, and will be traced at a constant rate
  of 100 millimeters per second. By this, and by using math to determine the 
  turning rate at 100mm/s, the constants TOTAL_TIME and TOTAL_FWD have been
  established and will be used to determine the time needed for iRobot to
  properly trace the polygon with desired specifications.
'''

'''
  According to the lab specifications, a button input is required to start/stop
  the motion of the iRobot. However, due to complications previously discussed
  (line 72) this was not possible. Instead, the iRobot will simply wait about ten
  seconds after initialization before starting to move, and cannot be stopped until
  it naturally finishes its polygon.
'''

try:
  N = sys.argv[1]
except:
  print("NO INPUT")

turnTime = TOTAL_TURN / N # Each turn should take 6.6s/N at a velocity of 100mm/s
driveTime = TOTAL_FWD / N # Each side of polygon should take 20.0s/N at a velocity of 100mm/s

connect = roombControl()
connect.setStart()
time.sleep(PAUSE)
connect.setSafe()
time.sleep(WAIT) # WANT IROBOT TO SLEEP FOR 10 SECONDS BEFORE ACTUALLY STARTING TO MOVE
  # This block establishes connection to iRobot and sets it to the proper state
for i in range(N):
  connect.drive(WHEEL_VEL, WHEEL_RAD_S, driveTime)
  connect.drive(WHEEL_VEL, WHEEL_RAD_T, turnTime)
    # The actual driving and making of the polygon. Simply drives and turns as necessary to complete polygon of N sides
connect.setStop() # After finishing driving, stop the Open Interface and terminate the connection to iRobot
  #End of program.
#
#
#
# ==== MAIN PROGRAM ====
#
#
#
