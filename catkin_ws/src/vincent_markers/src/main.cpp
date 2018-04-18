#include <stdio.h>
#include <stdarg.h>
#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <stack>
#include <math.h>

using namespace std;
using namespace ros;


float curr_x;
float curr_y;
float w, z;
float angle;
int mode = 0;
Publisher marker_pub;
Publisher a_star_marker;


stack<visualization_msgs::Marker> waypoints;

float cartesian_count (float x, float y, float x_target, float y_target) {
    float difference_x = x - x_target;
    float difference_y = y - y_target;
    return sqrt (difference_x*difference_x + difference_y*difference_y);
}


void pose_data_callback (const geometry_msgs::PoseStamped::Ptr& data) {
    curr_x = data->pose.position.x;
    curr_y = data->pose.position.y;
    w = data->pose.orientation.w;
    z = data->pose.orientation.z;

    float siny = 2.0 * (w * z);
    float cosy = 1.0 - (2.0 * z * z);
    angle = atan2 (siny, cosy);
    cout << angle * 180.0/3.141592653 +180.0<< endl;



    
    if (mode && !waypoints.empty()) {
        if (cartesian_count(curr_x, curr_y, waypoints.top().pose.position.x, waypoints.top().pose.position.y) <= 0.1) {
            //waypoints.top().action = visualization_msgs::Marker::DELETE;
            //marker_pub.publish (waypoints.top());
            
            cout << (waypoints.top().pose.position.x) << ", " << (waypoints.top().pose.position.y) << " popped" << endl;
            if (waypoints.top().ns == "vincent_sound_droppings") {
                // play music
                cout << "sound cue encountered" << endl;
            }

            waypoints.pop();
            // Play music here to0 
            
        }else {

            std_msgs::Float32MultiArray msg;
            vector<float> arr;

            // Angle
            float siny = 2.0 * (w * z);
            float cosy = 1.0 - (2.0 * z * z);
            angle = atan2 (siny, cosy);

            // Assume already rotated 
            siny = 2.0 * (waypoints.top().pose.orientation.w * waypoints.top().pose.orientation.z);
            cosy = 1.0 - (2.0 * waypoints.top().pose.orientation.z * waypoints.top().pose.orientation.z);
            float angle2 = atan2 (siny, cosy);
            angle2 += 180;
            angle2 = (int)angle2%360;
            int delta_angle = angle2 - angle;


            arr.push_back(delta_angle);
            arr.push_back(cartesian_count(waypoints.top().pose.position.x, waypoints.top().pose.position.y, curr_x, curr_y));
            msg.data = arr;
            a_star_marker.publish(msg); // Publish to be moved
        
            // Set next marker 
            visualization_msgs::Marker curr_marker;
            curr_marker = waypoints.top();
            marker_pub.publish (curr_marker); 

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
    // add a different marker namespace 
    curr_marker.pose.orientation.w = w;
    curr_marker.pose.orientation.z = z;
    
    curr_marker.scale.x = 0.2;
    curr_marker.scale.y = 0.2;
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

void cue_marker_sound (const std_msgs::Int32::Ptr& data) {
    visualization_msgs::Marker curr_marker;
    curr_marker.header.frame_id = "map";
    curr_marker.type = visualization_msgs::Marker::SPHERE;
    curr_marker.action = visualization_msgs::Marker::ADD;
    curr_marker.pose.position.x = curr_x;
    curr_marker.pose.position.y = curr_y;
    curr_marker.pose.position.z = 0;
    curr_marker.ns = "vincent_sound_droppings";
    // add a different marker namespace 
    curr_marker.pose.orientation.w = w;
    curr_marker.pose.orientation.z = z;
    

    
    curr_marker.scale.x = 0.2;
    curr_marker.scale.y = 0.2;
    curr_marker.scale.z = 0.5;
    curr_marker.color.a = 1.0; // Don't forget to set the alpha!
    curr_marker.color.r = 1.0;
    curr_marker.color.g = 1.0;
    curr_marker.color.b = 0.0;
    marker_pub.publish (curr_marker);
    

    waypoints.push(curr_marker);
    cout << curr_x << " " << curr_y << endl;
    cout << "Waypoint_sound pushed!" << endl;

}


int main (int argc, char **argv) {
    init(argc, argv, "vincent_markers");
    NodeHandle n;
    
    // Get current location
    Subscriber sub3 = n.subscribe("/slam_out_pose", 1, pose_data_callback); // This will update global map metadata
    marker_pub = n.advertise<visualization_msgs::Marker> ("markers", 1);
    a_star_marker = n.advertise<std_msgs::Float32MultiArray> ("markers_a_star", 1);
    Subscriber cue_sub = n.subscribe ("/cue_marker", 1, cue_callback);
    Subscriber pop_sub = n.subscribe ("/pop_markers", 1, cue_pop_callback);
    // Add a different marker namespace  
    Subscriber cue_sub_sound = n.subscribe ("/vincent_sound_markers", 1, cue_marker_sound);
    
    
    spin();
}
