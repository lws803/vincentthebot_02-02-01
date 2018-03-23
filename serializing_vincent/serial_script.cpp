#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include "serialising.h"

#define ROBOT_WIDTH 11.0

struct motor {
	float left, right;
}

void call_back(const geometry_msgs::Twist cmd-vel) {
	motor mtr;
	mtr.left = cmd_vel.linear.x - cmd_vel.angular.z * ROBOT_WIDTH/2;
	
	mtr.right = cmd_vel.linear.x + cmd_vel.angular.z * ROBOT_WIDTH/2
	
	
}

void listen() {
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000, call_back); 
	
	ros::spin();
	
}


