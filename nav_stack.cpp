#include <iostream>
#include <stdio.h>
#include <stdarg.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"
#include <math.h>
#include <list>
#include <map>
#include <set>


#define MAX 4000



int height = 0, width = 0;

using namespace std;
using namespace ros;

class coords_t {
public:
    int x, y;
    double g_cost;
    double h_cost;
    coords_t* parent;
    
    bool operator== (coords_t other) {
        if (other.x == this->x && other.y == this->y) {
            return true;
        }
        return false;
    }
    bool operator!= (coords_t other) {
        if (other.x != this->x && other.y != this->y) {
            return true;
        }
        return false;
    }
};


map<double, coords_t*> open; // Use this map to store coords_t and its f_cost, acts as a PQ
list<coords_t*> closed;
set<pair<int, int > > visited;

double cartesian_count (int x, int y, int x_target, int y_target) {
    double difference_x = x - x_target;
    double difference_y = y - y_target;
    if (mode) {
        return sqrt (difference_x*difference_x + difference_y*difference_y);
    }else {
        return abs(difference_x) + abs(difference_y);
    }
}


void print_maze (int8_t maze[][MAX]) {
    int i,d;
    for (i = 0; i < height; i++) {
        for (d = 0; d < width; d++) {
            printf("%2d ", maze[i][d]);
        }
        cout << endl;
    }
}

// TODO: Account for the unknown regions too 
void scan_neighbours (int8_t maze[][MAX],
                      coords_t* current, coords_t * start, coords_t * end) {
    int y = current->y, x = current->x;
    
    for (int i = -1; i < 2; i++) {
        for (int d = -1; d < 2; d++) {
            if (x+i < width && x+i >= 0 && y+d < height && y+d >= 0 && !(i == 0 && d == 0)) {
                coords_t *neighbour;
                if (x + i == end->x && y + d == end->y) {
                    // Just point the end to its parent
                    neighbour = end;
                    neighbour->g_cost = cartesian_count(x+i, y+d, start->x, start->y);
                    neighbour->h_cost = cartesian_count(x+i, y+d, end->x, end->y);
                    neighbour->parent = current;
                } else if (x + i == start->x && y + d == start->y) {
                    neighbour = start;
                    neighbour->g_cost = cartesian_count(x+i, y+d, start->x, start->y);
                    neighbour->h_cost = cartesian_count(x+i, y+d, end->x, end->y);
                    //neighbour->parent = current;
                }
                else {
                    neighbour = new coords_t;
                    neighbour->x = x+i;
                    neighbour->y = y+d;
                    neighbour->g_cost = cartesian_count(x+i, y+d, start->x, start->y);
                    neighbour->h_cost = cartesian_count(x+i, y+d, end->x, end->y);
                    neighbour->parent = current;
                }
                if (visited.find(make_pair(neighbour->x, neighbour->y)) == visited.end() && maze[y+d][x+i] <= probability ) {
                    open[neighbour->h_cost + neighbour->g_cost] = neighbour;
                    visited.insert(make_pair(neighbour->x, neighbour->y));
                }
            }
        }
    }
}

int pathfinder (coords_t * start, coords_t * end, int8_t maze[][MAX]) {
    open[0] = start;
    // Can only travel 4 directions
    while (!open.empty()) {
        coords_t *current = open.begin()->second;
        open.erase(open.begin());
        // Sorting
        
        closed.push_back(current);
        // pop(open, current);
        if (current->x == end->x && current->y == end->y) {
            return 1; // Path has been found
        }
        scan_neighbours (maze, current, start, end);
    }
    return 0;
}


// Modify this to provide way point management, if no waypoint, then it will stop and wait
void start_end_scan (coords_t *start, coords_t *end, int8_t maze[][MAX]) {
    // cout << "Enter start and end coords" << endl;
    // cout << "start: ";
    // cin >> start->x >> start->y;
    // cout << "end: ";
    // cin >> end->x >> end->y;
    
    // Dummy values 
    start->x = 1; start->y = 1;
    end->x = 10; end->y = 10;
    
    start->g_cost = 0;
    start->h_cost = cartesian_count(start->x, start->y, end->x, end->y);
    end->h_cost = 0;
    end->g_cost = cartesian_count(start->x, start->y, end->x, end->y);
}

int find_heading (coords_t * start, int8_t maze[][MAX]) {
    for (int i = -1; i < 2; i++) {
        for (int d = -1; d < 2; d++) {
            if (!(i == 0 && d == 0) && maze[start->y + d][start->x + i] == -3) {
                // Found heading
                if (i == 0 && d == -1) return 0; // north
                else if (i == 0 && d == 1) return 180; // south
                else if (i == -1 && d == 0) return 270; // East
                else if (i == 1 && d == 0) return 90; // West
                else if (i == 1 && d == -1) return 45; // North West
                else if (i == 1 && d == 1) return 135; // South West
                else if (i == -1 && d == 1) return 225; // South East
                else if (i == -1 && d == -1) return 315; // North West
            }
        }
    }
    return -1;
}


// Params to set
int probability = 4;
int mode = 1; // 0 means manhattan distance, 1 means cartesian distance
// Need to juggle between the two, sometimes its actually better to use this
// Manhattan distance cannot travel diagonally


// TODO: Get map meta data here too from a global variable 
void generate_path (int maze[][MAX]) {    
    
    coords_t *start = new coords_t;
    coords_t *end = new coords_t;
    int steps = 1;
    
    start_end_scan(start, end, maze);
    
    cout << "Original Maze:" << endl;
    print_maze(maze);
    
    pathfinder(start, end, maze);
    
    // Traceback
    maze[end->y][end->x] = -3; // We use -4 to signify the end
    coords_t* current_point = end;
    int curr_heading = find_heading(current_point, maze);
    int steps_start = 2;
    
    // Iterate thru the next few
    while (current_point != start) {
        // Continue tracing
        current_point = current_point->parent;
        int temp_heading = find_heading(current_point, maze);
        if (temp_heading == curr_heading) steps_start++;
        else {
            // Reset steps
            steps_start = 2;
            curr_heading = temp_heading;
        }
        
        maze[current_point->y][current_point->x] = -3;
        steps++;
    }
    
    
    cout << "Shortest path:" << endl << endl;
    print_maze (maze);
    cout << "Heading: " << find_heading(start, maze) << endl;
    cout << "Total distance to next junction: " << steps_start << endl;
    cout << "Total steps: " << steps << endl;
}


NodeHandle n;


void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& data)
{
    // Receive data here
    generate_path(data->data);
    
    // After receiving, then publish or activate motor here or pub to /movement, twist
    // Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/movement", 2);
}

void map_meta_data_callback (const nav_msgs::MapMetaData::ConstPtr& data) {
	// Note that these two might be floating point numbers, do we have to do some sort of conversion? 
	// TODO
	height = data->height;
	width = data->width;
}



int main(int argc, char **argv){
    init(argc, argv, "front_cam");
    
    
    Subscriber sub = n.subscribe("/map", 1, map_callback); // This will call A* itself and control the systems  
    Subscriber sub2 = n.subscribe("/map_metadata", 1, map_meta_data_callback); // This will update global map metadata
    // TODO: Add another subscriber to handle slam_out_pose 


    spin();
}
