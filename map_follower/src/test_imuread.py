import rospy
from std_msgs.msg import String
import serial

# Enable serial 
console_ser = serial.Serial(port = "/dev/ttyACM0", baudrate=115200)
console_ser.close()
console_ser.open()
initread = console_ser.read(62)
console_ser.write('IMU0')
console_ser.write('IMU1')
#initread = console_ser.read(50)
print initread

def imu_talker():
    imupub = rospy.Publisher('imu_read', String, queue_size=10)
    rospy.init_node('imu_talker', anonymous=True)
    rate = rospy.Rate(150) # 10hz
    while not rospy.is_shutdown():
        if console_ser.inWaiting()>0:
            imuread = console_ser.read(22)
	    print("IMU read", imuread)
    	rospy.loginfo(imuread)
    	imupub.publish(imuread)
    	rate.sleep()

if __name__ == '__main__':
    imu_talker()
