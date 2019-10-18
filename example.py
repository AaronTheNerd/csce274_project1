import serial
import time
from struct import pack, unpack

def unwrap(raw_data, v1, v2):
    ret_list = []
    for index in range(len(raw_data)):
        if raw_data[index] == v1 and raw_data[(index + 1) % len(raw_data)] == v2:
            ret_list = [raw_data[(i + index - 1) % len(raw_data)] for i in raw_data]
            break
    return ret_list

if __name__ == "__main__":
    connection = serial.Serial('/dev/ttyUSB0', baudrate=115200)
    time.sleep(0.1)
    START = chr(128)
    connection.write(START)
    time.sleep(0.1)
    SAFE = chr(131)
    connection.write(SAFE)
    time.sleep(0.1)
    BUTTONS = 18
    PACKETS = {BUTTONS : 1}
    num_of_packets = len(PACKETS)
    num_of_bytes = sum(PACKETS.values()) + num_of_packets + 3
    READ_SENSORS = 148
    com = pack('>2B', READ_SENSORS, num_of_packets)
    com += pack('%sB' % num_of_packets, *PACKETS.keys)
    connection.write(com)
    time.sleep()
    read_com = '>' + str(num_of_bytes) + 'B'
    while True:
        raw_data = unpack(read_com, connection.read(size=num_of_bytes)
        print "Raw Data:", raw_data
        raw_data = unwrap(raw_data, 19, num_of_bytes - 3)
        print "Unwrapped Raw Data:", raw_data