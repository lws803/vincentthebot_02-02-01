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


int main (int argc, char **argv) {

    init(argc, argv, "marker_helper");
    NodeHandle n;

    marker_pop = n.advertise<std_msgs::Int32> ("pop_markers", 1);
    marker_cue = n.advertise<std_msgs::Int32> ("cue_marker", 1);
    cout << "0 to plant waypoints, 1 to start popping" << endl; 

    while(true) {
        int command;
        cin >> command;
        std_msgs::Int32 msg; 
        msg.data = 0;
        if (command == 0) {
            marker_cue.publish (msg);
        }else {
            marker_pop.publish (msg);
        }
    }


}