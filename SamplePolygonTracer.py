from serial import Serial
from time import sleep
from struct import pack
CONN = Serial(port='/dev/ttyUSB0', baudrate=115200)

'''
  Need to do:
  implement this using interface
  Take input from button input
  does input for  polygon sides need to be button presses or user input via command?
'''

'''CONN.write(chr(128))
sleep(1)
CONN.write(chr(131))
sleep(1)
CONN.write(pack('>B2h', 137, 100, 0))
sleep(2)
CONN.write(pack('>B2h', 137, 0, 0))'''


TOTAL_TURN=6.67 #CONSTANT VALUE FOR CALCULATIONS
TOTAL_FWD=20
PAUSE=0.5


def clear():  # Sets roomba to safe mode, needs to be called frequently
  sleep(PAUSE)
  CONN.write(chr(128))
  sleep(PAUSE)
  CONN.write(chr(131))

def drive(s): #Drive in a straight line for (s) seconds
  clear()
  #print("executing drive")
  sleep(PAUSE)
  CONN.write(pack('>B2h',137,100,0))
  sleep(s)
  CONN.write(pack('>B2h',137,0,0))
  sleep(PAUSE)

def turn(s):  #Turn counterclockwise for (s) seconds
  clear()
  #print("executing turn")
  sleep(PAUSE)
  CONN.write(pack('>B2h',137,100,1))
  sleep(s)
  CONN.write(pack('>B2h',137,0,1))
  sleep(PAUSE)

def shape(x): #Simple function which can take an integer input (x), then Roomba traces a polygon with (x) number of sides and vertexes
  clear()
  sleep(PAUSE)
  turnTime=TOTAL_TURN / x
  #print(turnTime)
  driveTime = TOTAL_FWD / x
  sleep(PAUSE)
  #print(driveTime)
  for i in range(x):
    #print('trial')
    drive(driveTime)
    sleep(PAUSE)
    turn(turnTime)


kb = input("Enter number of sides")

clear()
shape(kb)
