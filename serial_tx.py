#!/usr/bin/env python

import rospy
import serial
import locale
from locale import atof
from std_msgs.msg import String, Int8, Float32
from geometry_msgs.msg import Twist

VEL_ERROR_EST_FACTOR =  1.0
L_ERROR_EST_FACTOR =    1.0

SAMPLE_TIME = 4.0 #thoi gian lay mau (ms)
CONVERT_FACTOR = 2630.85018 #15167.54 #18554.4170589864          #545.5552478538       

ALPHA = CONVERT_FACTOR * VEL_ERROR_EST_FACTOR * (SAMPLE_TIME / 1000.0)

DELAY_TIME = 200
L = 0.32 * L_ERROR_EST_FACTOR
PHI = L*0.5

str_msg_tx = "0.0/0.0k"
flag_uart = 0x0
enc_L = 0.0
enc_R = 0.0
linear_x = 0.0
angular_z = 0.0

def MOTOR_Enc_Cal():
    global enc_L, enc_R, linear_x, angular_z

    v_L = linear_x - angular_z * PHI
    v_R = linear_x + angular_z * PHI

    enc_L = v_L * ALPHA
    enc_R = v_R * ALPHA

    print(enc_L)
    print(enc_R)

def Callback(vel_sub_msg):
    global str_msg_tx
    global flag_uart 
    global enc_L, enc_R, linear_x, angular_z

    linear_x= vel_sub_msg.linear.x
    angular_z= vel_sub_msg.angular.z

    MOTOR_Enc_Cal()
    str_msg_tx = "%.2f/%.2fk" % (enc_L,enc_R)
    flag_uart = 0x1

def main():
    port_name = rospy.get_param('~port','/dev/ttyAMA0')
    print(port_name)
    baud = int(rospy.get_param('~baud','115200'))

    ser = serial.Serial(port = port_name, baudrate = baud, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS, timeout=1) 

    rospy.init_node('serial_tx', anonymous=True)
    rospy.Subscriber('/cmd_vel',Twist,Callback)

    global str_msg_tx
    global flag_uart

    while not rospy.is_shutdown():
        if flag_uart == 0x1:
            flag_uart = 0x0
            ser.write(str_msg_tx)
            rospy.loginfo(str_msg_tx)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
