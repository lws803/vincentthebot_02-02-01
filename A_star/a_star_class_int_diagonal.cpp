//
//  main.cpp
//  maze_astar
//
//  Created by Ler Wilson on 17/1/18.
//  Copyright © 2018 Ler Wilson. All rights reserved.
//

#include <iostream>
#include <math.h>
#include <list>
#include <map>
#include <set>

#define MAX 11

using namespace std;

class coords_t {
public:
    int x, y;
    double g_cost;
    double h_cost;
    coords_t* parent;
    // Constructor
    coords_t() {
    }
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
private:
};


class A_star {
private:
    
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
        for (i = 0; i < MAX; i++) {
            for (d = 0; d < MAX; d++) {
                printf("%2d ", maze[i][d]);
            }
            cout << endl;
        }
    }
    
    
    void scan_neighbours (int8_t maze[][MAX],
                          coords_t* current, coords_t * start, coords_t * end) {
        int y = current->y, x = current->x;
        
        for (int i = -1; i < 2; i++) {
            for (int d = -1; d < 2; d++) {
                if (x+i < MAX && x+i >= 0 && y+d < MAX && y+d >= 0 && !(i == 0 && d == 0)) {
                    coords_t *neighbour;
                    if (x + i == end->x && y + d == end->y) {
                        // Just point the end to its parent
                        neighbour = end;
                        neighbour->g_cost = cartesian_count(x+i, y+d, start->x, start->y);
                        neighbour->h_cost = cartesian_count(x+i, y+d, end->x, end->y);
                        neighbour->parent = current;
                    }else {
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
        cout << "Enter start and end coords" << endl;
        cout << "start: ";
        cin >> start->x >> start->y;
        cout << "end: ";
        cin >> end->x >> end->y;
        
        
        
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

public:
    // Params to set
    int probability = 4;
    int mode = 1; // 0 means manhattan distance, 1 means cartesian distance
    // Need to juggle between the two, sometimes its actually better to use this
    
    
    A_star (int h, int w) {
        // height and width of map
        // Accept the map here
    }
    A_star () {}
    
    ~A_star () {
        // Add deletion here
    }
    
    void generate_path () {
        int8_t maze[MAX][MAX] = {
            {5,5,5,5,5,5,5,5,5,5,5},
            {5,0,0,0,0,0,0,0,0,0,5},
            {5,0,0,0,0,0,0,0,0,0,5},
            {5,0,0,0,0,0,0,0,0,0,5},
            {5,0,0,0,0,0,0,0,0,0,5},
            {5,0,0,0,0,0,0,0,0,0,5},
            {5,0,0,0,0,0,0,0,0,0,5},
            {5,0,0,0,0,0,0,0,0,0,5},
            {5,0,0,0,0,0,0,0,0,0,5},
            {5,0,0,0,0,0,0,0,0,0,5},
            {5,5,5,5,5,5,5,5,5,5,5}};
        
        
        coords_t *start = new coords_t;
        coords_t *end = new coords_t;
        int steps = 1;
        // We can disable scan_maze
        
        start_end_scan(start, end, maze);
        
        cout << "Original Maze:" << endl;
        print_maze(maze);
        
        // Initial run
        scan_neighbours (maze, start, start, end);
        pathfinder(start, end, maze);
        
        // Traceback
        maze[end->y][end->x] = -3; // We use 9 to signify the path
        coords_t* current_point = end;
        
        // Iterate thru the next few
        while (current_point->x != start->x || current_point->y != start->y) {
            // Continue tracing
            current_point = current_point->parent;
            maze[current_point->y][current_point->x] = -3;
            steps++;
            // TODO: Count longest sustainable sequence here as well as this will ultimately lead us to count distance from start to junction
        }
        cout << "Shortest path:" << endl << endl;
        print_maze (maze);
        cout << "Heading: " << find_heading(start, maze) << endl;
        cout << "Total steps: " << steps << endl;
        
    }
    
};


int main(int argc, const char * argv[]) {
    A_star map;
    map.generate_path();
    
    return 0;
}

// TODO: Set it up such that it can receive a dynamic map from hector slam and process it then output the shortest path based on a fixed destination on the map



