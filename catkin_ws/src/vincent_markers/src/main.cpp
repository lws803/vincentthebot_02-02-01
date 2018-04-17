#include <stdio.h>
#include <stdarg.h>
#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <stack>

using namespace std;
using namespace ros;


int curr_x;
int curr_y;
int mode = 0;
Publisher marker_pub;
Publisher a_star_marker;


stack<visualization_msgs::Marker> waypoints;

double cartesian_count (int x, int y, int x_target, int y_target) {
    double difference_x = x - x_target;
    double difference_y = y - y_target;
    return sqrt (difference_x*difference_x + difference_y*difference_y);
}


void pose_data_callback (const geometry_msgs::PoseStamped::Ptr& data) {
    curr_x = data->pose.position.x;
    curr_y = data->pose.position.y;
    //cout << curr_x << endl;
    //cout << curr_y << endl;
    
    if (mode && !waypoints.empty()) {
        if (cartesian_count(curr_x, curr_y, waypoints.top().pose.position.x, waypoints.top().pose.position.y) <= 1) {
            waypoints.top().action = visualization_msgs::Marker::DELETE;
            marker_pub.publish (waypoints.top());
            
            cout << (waypoints.top().pose.position.x) << ", " << (waypoints.top().pose.position.y) << " popped" << endl;
            waypoints.pop();
            // Play music here too
            
        }else {
            std_msgs::Int32MultiArray msg;
            vector<int> arr;
            arr.push_back((int)waypoints.top().pose.position.x);
            arr.push_back((int)waypoints.top().pose.position.y);
            msg.data = arr;
            a_star_marker.publish(msg);
        }
    }
}



void cue_callback (const std_msgs::Int32::Ptr& data) {
    visualization_msgs::Marker curr_marker;
    curr_marker.header.frame_id = "map";
    curr_marker.type = visualization_msgs::Marker::SPHERE;
    curr_marker.action = visualization_msgs::Marker::ADD;
    curr_marker.pose.position.x = curr_x;
    curr_marker.pose.position.y = curr_y;
    curr_marker.pose.position.z = 0;
    curr_marker.ns = "vincent_droppings";
    curr_marker.pose.orientation.w = 1.0;
    
    
    curr_marker.scale.x = 0.5;
    curr_marker.scale.y = 0.5;
    curr_marker.scale.z = 0.5;
    curr_marker.color.a = 1.0; // Don't forget to set the alpha!
    curr_marker.color.r = 0.0;
    curr_marker.color.g = 1.0;
    curr_marker.color.b = 0.0;
    marker_pub.publish (curr_marker);
    
    waypoints.push(curr_marker);
    cout << curr_x << " " << curr_y << endl;
    cout << "Waypoint pushed!" << endl;
}


void cue_pop_callback (const std_msgs::Int32::Ptr& data) {
    mode = 1;
    cout << "set to pop markers.. " << endl;
}



int main (int argc, char **argv) {
    init(argc, argv, "vincent_markers");
    NodeHandle n;
    
    // Get current location
    Subscriber sub3 = n.subscribe("/slam_out_pose", 1, pose_data_callback); // This will update global map metadata
    marker_pub = n.advertise<visualization_msgs::Marker> ("markers", 1);
    a_star_marker = n.advertise<std_msgs::Int32MultiArray> ("markers_a_star", 1);
    Subscriber cue_sub = n.subscribe ("/cue_marker", 1, cue_callback);
    Subscriber pop_sub = n.subscribe ("/pop_markers", 1, cue_pop_callback);

    
    
    spin();
}
