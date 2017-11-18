#!/usr/bin/env python
# import the serial package to use the hardware serial ports
import serial
import time
# initialize serial port to communicate with Tri-Car
#We are using a USB to RS232 converter on ttyUSB0 for Tri-Car
console_ser = serial.Serial(port = "/dev/ttyACM0", baudrate=115200)
# the port can be changed if there are multiple USB devices
#commands to open and close the serial port
console_ser.close()
console_ser.open()
#construct and send the Ackermann steering commands to rally car 
for i in range(-2048, 0, 1):
    console_ser.write("A"+"%05d" %i+"+0200")

for j in range(2049):
    console_ser.write("A"+"%04d" %j+"+0400")

console_ser.close()


