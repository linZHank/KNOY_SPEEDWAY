from __future__ import print_function
import rospy
import serial

def clean_shutdown():
    print("Stopping sending the command...")
    console_ser.write('A+0000+0000')
    console_ser.close()
    return True

console_ser = serial.Serial(port = '/dev/ttyACM0', baudrate=115200)
console_ser.close()
console_ser.open()

def main():
    rospy.init_node('test_serialcommand')
    for i in range(5000):
        console_ser.write('A+1000+0500')
        print("iter: ", i)
    console_ser.write('A+0000+0000')
    rospy.on_shutdown(clean_shutdown)
    rospy.spin()

if __name__=='__main__':
    main()
