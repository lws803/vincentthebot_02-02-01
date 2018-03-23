#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import serial 


def callback(cmd_vel):
    rospy.loginfo (cmd_vel)
    # Create a struct 
	'''
	struct motor {
		float left;
		float right;	
	};

	'''
	left_speed_out = cmd_vel.linear.x - cmd_vel.angular.z * ROBOT_WIDTH/2
	right_speed_out = cmd_vel.linear.x + cmd_vel.angular.z * ROBOT_WIDTH/2
	

	

	# Serial write here - send the struct out as a string 



def listen():
	rospy.init_node('cmd_listener', anonymous=True)
	rospy.Subscriber('cmd_vel', Twist, callback)

	rospy.spin()



if __name__ == '__main__':
	port = serial.Serial(port = "/dev/tty*")

	if (not port.isOpen):
		port.open()

	listen()

    rospy.spin()
