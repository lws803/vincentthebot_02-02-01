#include <stdio.h>
#include <stdarg.h>
#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <iostream>
#include <stack>


using namespace std;
using namespace ros;

Publisher marker_pop;
Publisher marker_cue;
Publisher marker_cue_sound;

int main (int argc, char **argv) {

    init(argc, argv, "marker_helper");
    NodeHandle n;

    marker_pop = n.advertise<std_msgs::Int32> ("pop_markers", 1);
    marker_cue = n.advertise<std_msgs::Int32> ("cue_marker", 1);
    marker_cue_sound = n.advertise<std_msgs::Int32> ("vincent_sound_markers", 1);
    

    cout << "0 to plant waypoints, 1 to plant sound waypoints, 2 to start popping," << endl; 
    // Set it such that the original Marker will be placed everytime vincent stops 
    while(true) {
        int command;
        cin >> command;
        std_msgs::Int32 msg; 
        msg.data = 0;
        if (command == 0) {
            marker_cue.publish (msg);
        }else if (command == 1){
            marker_cue_sound.publish(msg);
        }else {
            marker_pop.publish (msg);
            break;
        }
    }


}