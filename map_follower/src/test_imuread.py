import rospy
from std_msgs.msg import String
import serial

# Enable serial 
console_ser = serial.Serial(port = "/dev/ttyACM0", baudrate=115200)
console_ser.close()
console_ser.open()
console_ser.read(32)
console_ser.write('IMU0')
console_ser.write('IMU1')

def imu_talker():
    imupub = rospy.Publisher('imu_read', String, queue_size=10)
    rospy.init_node('imu_talker', anonymous=True)
    rate = rospy.Rate(150) # 10hz
    while not rospy.is_shutdown():
        if console_ser.inWaiting()>0:
            read = console_ser.read(19)
	    print("IMU read", read)
    	rospy.loginfo(read)
    	imupub.publish(read)
    	rate.sleep()

if __name__ == '__main__':
    imu_talker()
