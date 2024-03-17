import serial
import struct
import time

with serial.Serial("/dev/ttyUSB1", baudrate=115200, timeout=1) as ser:
    try:
        while True:
            ser.write(b"\x01")
            l = ser.readline()
            print(l.decode())

    except Exception as e:
        pass