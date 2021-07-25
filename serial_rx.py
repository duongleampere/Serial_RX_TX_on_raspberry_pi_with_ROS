#!/usr/bin/env python

import rospy
import serial
import locale
from locale import atof
from std_msgs.msg import String, Int8, Float32
from geometry_msgs.msg import Twist, Quaternion

VEL_ERROR_EST_FACTOR =  1.0
L_ERROR_EST_FACTOR =    1.0

SAMPLE_TIME = 4.0 #thoi gian lay mau (ms)
CONVERT_FACTOR = 2630.85018#15167.54 #18554.4170589864      #545.5552478538     

ALPHA = CONVERT_FACTOR * VEL_ERROR_EST_FACTOR * SAMPLE_TIME / 1000.0
BETA = 1 / ALPHA
DELAY_TIME = 200
SAMPLE_TIME = 4.0 #thoi gian lay mau (ms)

L = 0.32 * L_ERROR_EST_FACTOR

str_msg_rx = "0.0 0.0 1.0 0.0 0.0 0.0" #frame data nhan tu STM32F4 discovery
flag_uart = 0x0
vel_pub_msg = Twist() #van toc
# quat_pub_msg = Quaternion()
theta_pub_msg = Float32() #goc theta
enc_L = 0.0 
enc_R = 0.0
v_L = 0.0
v_R = 0.0

def doNothing():
	return 

def MOBILE_Vel_Cal():
	global v_L, v_R, enc_L, enc_R
	global vel_pub_msg

	v_L = BETA * enc_L 
	v_R = BETA * enc_R

	vel_pub_msg.linear.x = (v_L + v_R)/2.0
	vel_pub_msg.angular.z = (v_R - v_L)/L


def main():
	port_name = rospy.get_param('~port','/dev/ttyAMA0')
	baud = int(rospy.get_param('~baud','115200'))

	ser = serial.Serial(port = port_name, baudrate = baud, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS, timeout=1) 
	# if ~(ser.is_open):
	#     ser.open()

	rospy.init_node('serial_rx', anonymous=True) #tao node co ten serial
	vel_pub = rospy.Publisher('vel_pub', Twist, queue_size=10)
	# quat_pub = rospy.Publisher('quat_pub', Quaternion, queue_size=10)
	theta_pub = rospy.Publisher('theta_pub', Float32, queue_size=10)   

	global str_msg_rx
	global flag_uart
	global vel_pub_msg
	global enc_L, enc_R

	i = 0
	tmp = str_msg_rx.split('/')

	while not rospy.is_shutdown():
		i += 1
		x = ser.readline()     
		if len(x) >= 3: #neu khong doc duoc gi tu serial rx
			tmp = str_msg_rx.split('/')
			try:
				enc_L = float(tmp[0])
				enc_R = float(tmp[1])
				# quat_pub_msg.w = float(tmp[2])
				# quat_pub_msg.x = float(tmp[3])
				# quat_pub_msg.y = float(tmp[4])
				# quat_pub_msg.z = float(tmp[5])
				theta_pub_msg.data = float(tmp[2]) #IMU
				#

				MOBILE_Vel_Cal()
			except:
				rospy.loginfo(str_msg_rx)

		# 	str_msg_rx = ""         
		# else:
		# 	str_msg_rx += x

		if i == 20:
			i = 0
			vel_pub.publish(vel_pub_msg)
			# quat_pub.publish(quat_pub_msg)
			theta_pub.publish(theta_pub_msg)

		# rate.sleep()
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
