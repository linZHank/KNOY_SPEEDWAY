import rospy
from std_msgs.msg import String
import serial

# Enable serial 
console_ser = serial.Serial(port = "/dev/ttyACM0", baudrate=115200)
console_ser.close()
console_ser.open()
console_ser.read(60)
console_ser.write('IMU1')

def imu_talker():
    imupub = rospy.Publisher('imu_read', String, queue_size=10)
    rospy.init_node('imu_talker', anonymous=True)
    rate = rospy.Rate(150) # 10hz
    while not rospy.is_shutdown():
        if console_ser.inWaiting()>0:
            read = console_ser.read(36)
	    print("IMU read", read)
	    if 'I' in read and 'U' in read:
		s = read.find('I')
		e = read.find('U')
		car_ser = read[s:e+1]
    	rospy.loginfo(car_ser)
    	imupub.publish(car_ser)
    	rate.sleep()

if __name__ == '__main__':
    imu_talker()
