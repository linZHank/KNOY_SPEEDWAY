#!/usr/bin/env python
# import the serial package to use the hardware serial ports
import serial
import time
import pdb
# initialize serial port to communicate with Tri-Car
#We are using a USB to RS232 converter on ttyUSB0 for Tri-Car
console_ser = serial.Serial(port = "/dev/ttyACM0", baudrate=115200)
# the port can be changed if there are multiple USB devices
#commands to open and close the serial port
console_ser.close()
console_ser.open()
#construct and send the Ackermann steering commands to rally car 
console_ser.read()
console_ser.write("IMU1")
while True:    # time.sleep(0.1)
    if console_ser.inWaiting()>0:
        read = console_ser.read(20)
        #time.sleep(0.2)
        print(read)
    time.sleep(1)
# console_ser.close()


