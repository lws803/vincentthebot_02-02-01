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
#include <vector>
#include <queue>


using namespace std;
using namespace ros;


int curr_x = 1, curr_y = 1;


class a_star
{
private:
	vector<vector<int> > maze; 

	struct coords_t
	{
		int x, y;
	    double g_cost;
	    double h_cost;
	    coords_t* parent;

	};

	//map<double, coords_t* > open; // Use this map to store coords_t and its f_cost, acts as a PQ

	list <coords_t*> open;
	int height = 0, width = 0;

	// To be uodated by call_back

	list<coords_t*> closed;
	set<pair<int, int > > visited;

	// Params to set
	int probability = 4;
	int mode = 1; // 0 means manhattan distance, 1 means cartesian distance
	// Need to juggle between the two, sometimes its actually better to use this
	// Manhattan distance cannot travel diagonally



public:
	a_star(const nav_msgs::OccupancyGrid::Ptr &data, int height, int width) {
		this->height = height;
		this->width = width;
		int index = 0;

	    for (int i = 0; i < height; ++i)
	    {
	    	vector<int> temp;
	    	for (int d = 0; d < width; ++d)
	    	{
	    		temp.push_back((int)data->data[index++]);
	    	}
	    	maze.push_back(temp);
	    }

	}
	~a_star() {
		//delete [] maze;
	}


	double cartesian_count (int x, int y, int x_target, int y_target) {
	    double difference_x = x - x_target;
	    double difference_y = y - y_target;
	    if (mode) {
	        return sqrt (difference_x*difference_x + difference_y*difference_y);
	    }else {
	        return abs(difference_x) + abs(difference_y);
	    }
	}


	void print_maze () {
	    int i,d;
	    for (i = 0; i < height; i++) {
	        for (d = 0; d < width; d++) {
	            printf("%2d ", maze[height][width]);
	        }
	        cout << endl;
	    }
	}

	// TODO: Account for the unknown regions too 
	void scan_neighbours ( coords_t* current, coords_t * start, coords_t * end) {
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
	                if (visited.find(make_pair(neighbour->x, neighbour->y)) == visited.end() && maze[(y+d)][(x+i)] <= probability ) {
	                    //open[neighbour->h_cost + neighbour->g_cost] = neighbour;
	                    open.push_back(neighbour);
	                    visited.insert(make_pair(neighbour->x, neighbour->y));
	                }
	            }
	        }
	    }
	}

	int pathfinder (coords_t * start, coords_t * end) {
	    //open[0] = start;

	    open.push_back (start);
	    // Can only travel 4 directions
	    while (!open.empty()) {
            coords_t *current = open.front();
            // Sorting
            list<coords_t*>::iterator iterator;
            for (iterator = open.begin(); iterator != open.end(); ++iterator) {
                if (current->h_cost + current->g_cost > (*iterator)->g_cost + (*iterator)->h_cost) {
                    current = *iterator; // Set current to node with the lowest f_cost
                }else if (current->h_cost + current->g_cost == (*iterator)->g_cost + (*iterator)->h_cost &&
                          current->h_cost > (*iterator)->h_cost) {
                    // If they have the same f_cost
                    current = *iterator;
                }
            }

            open.remove (current);
	        closed.push_back(current);

	        if (current->x == end->x && current->y == end->y) {
	            return 1; // Path has been found
	        }
	        scan_neighbours (current, start, end);
	    }
	    return 0;
	}


	// Modify this to provide way point management, if no waypoint, then it will stop and wait
	void start_end_scan (coords_t *start, coords_t *end) {    
	    // Dummy values 

	    start->x = curr_x + width/2; start->y = curr_y + height/2;
	    end->x = 10; end->y = 10;
	    
	    start->g_cost = 0;
	    start->h_cost = cartesian_count(start->x, start->y, end->x, end->y);
	    end->h_cost = 0;
	    end->g_cost = cartesian_count(start->x, start->y, end->x, end->y);
	}

	int find_heading (coords_t * start) {
	    for (int i = -1; i < 2; i++) {
	        for (int d = -1; d < 2; d++) {
	            if (!(i == 0 && d == 0) 
	            	&& start->x + i < width && start->x + i >= 0
	            	&& start->y + d < height && start->y + d >= 0
	            	&& maze[start->y + d][start->x + i] == -3) {
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


	// TODO: Get map meta data here too from a global variable 
	void generate_path () {    

	    coords_t *start = new coords_t;
	    coords_t *end = new coords_t;
	    int steps = 1;
	    
	    start_end_scan(start, end);
	    cout << "Passed start end scan" << endl;
	    
	  	
	    pathfinder(start, end);
	    cout << "Passed A*" << endl;


	    // Traceback
	    maze[(end->y)][(end->x)] = -3; // We use -4 to signify the end
	    coords_t* current_point = end;
	    int curr_heading = find_heading(current_point);
	    int steps_start = 2;
	    

	    cout << "start " << start->x << " " << start->y << endl;
	  	cout << "end " << end->x << " " << end->y << endl;

	   	//print_maze ();
	   	// TODO: Find out what's wrong with the traversal of the maze 
	    // Iterate thru the next few
	    while (current_point != start) {
	        // Continue tracing
	        current_point = current_point->parent;
	        int temp_heading = find_heading(current_point);
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
	    //print_maze (maze);
	    cout << "Heading: " << find_heading(start) << endl;
	    cout << "Total distance to next junction: " << steps_start << endl;
	    cout << "Total steps: " << steps << endl;
	}




	
};

void pose_data_callback (const geometry_msgs::PoseStamped::Ptr& data) {
	// TODO: might need to import geometry_msgs/Pose.h
	curr_x = data->pose.position.x;
	curr_y =data->pose.position.y;
	//cout << data->pose.position.x << endl;
}




void map_callback(const nav_msgs::OccupancyGrid::Ptr &data)
{
    // Receive data here
    //generate_path(data->data[0]);

    // TODO: Find out what caused the seg fault
    a_star graph(data, data->info.height, data->info.width);

    graph.generate_path();

    // After receiving, then publish or activate motor here or pub to /movement, twist
    // Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/movement", 2);
}

int main(int argc, char **argv){
    init(argc, argv, "nav_stack");

    NodeHandle n;

    
    Subscriber sub = n.subscribe("/map", 1, map_callback); // This will call A* itself and control the systems  
    Subscriber sub3 = n.subscribe("/slam_out_pose", 1, pose_data_callback); // This will update global map metadata


    spin();
}


// TODO: make map call back update the local buffer instead of passing it straight.

// Make A* perform on each pose update and see how fast it can go 