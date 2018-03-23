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
        return sqrt (difference_x*difference_x + difference_y*difference_y);
    }
    
    
    void print_maze (int8_t maze[][MAX]) {
        int i,d;
        for (i = 0; i < MAX; i++) {
            for (d = 0; d < MAX; d++) {
                cout << (int)maze[i][d] << " ";
            }
            cout << endl;
        }
    }
    
    
    void scan_neighbours (int8_t maze[][MAX],
                          coords_t* current, coords_t start, coords_t end) {
        int y = current->y, x = current->x;
        if (y+1 < MAX) {
            coords_t *neighbour = new coords_t;
            neighbour->x = x;
            neighbour->y = y+1;
            neighbour->g_cost = cartesian_count(x, y+1, start.x, start.y);
            neighbour->h_cost = cartesian_count(x, y+1, end.x, end.y);
            neighbour->parent = current;
            
            if (visited.find(make_pair(neighbour->x, neighbour->y)) == visited.end() && maze[y+1][x] <= probability ) {
                open[neighbour->h_cost + neighbour->g_cost] = neighbour;
                visited.insert(make_pair(neighbour->x, neighbour->y));
            }
        }
        if (y-1 >= 0) {
            coords_t *neighbour = new coords_t;
            neighbour->x = x;
            neighbour->y = y-1;
            neighbour->g_cost = cartesian_count(x, y-1, start.x, start.y);
            neighbour->h_cost = cartesian_count(x, y-1, end.x, end.y);
            neighbour->parent = current;
            if (visited.find(make_pair(neighbour->x, neighbour->y)) == visited.end() && maze[y-1][x] <= probability) {
                open[neighbour->h_cost + neighbour->g_cost] = neighbour;
                visited.insert(make_pair(neighbour->x, neighbour->y));
            }
        }
        if (x-1 >= 0) {
            coords_t *neighbour = new coords_t;
            neighbour->x = x-1;
            neighbour->y = y;
            neighbour->g_cost = cartesian_count(x-1, y, start.x, start.y);
            neighbour->h_cost = cartesian_count(x-1, y, end.x, end.y);
            neighbour->parent = current;
            if (visited.find(make_pair(neighbour->x, neighbour->y)) == visited.end() && maze[y][x-1] <= probability) {
                open[neighbour->h_cost + neighbour->g_cost] = neighbour;
                visited.insert(make_pair(neighbour->x, neighbour->y));
                
            }
        }
        if (x+1 < MAX) {
            coords_t *neighbour = new coords_t;
            neighbour->x = x+1;
            neighbour->y = y;
            neighbour->g_cost = cartesian_count(x+1, y, start.x, start.y);
            neighbour->h_cost = cartesian_count(x+1, y, end.x, end.y);
            neighbour->parent = current;
            if (visited.find(make_pair(neighbour->x, neighbour->y)) == visited.end() && maze[y][x+1] <= probability) {
                open[neighbour->h_cost + neighbour->g_cost] = neighbour;
                visited.insert(make_pair(neighbour->x, neighbour->y));
            }
        }
    }
    
    int pathfinder (coords_t start, coords_t end, int8_t maze[][MAX]) {
        // Can only travel 4 directions
        while (!open.empty()) {
            coords_t *current = open.begin()->second;
            open.erase(open.begin());
            // Sorting
            
            closed.push_back(current);
            // pop(open, current);
            if (current->x == end.x && current->y == end.y) {
                return 1; // Path has been found
            }
            scan_neighbours (maze, current, start, end);
        }
        return 0;
    }
    
    
    // Modify this to provide way point management, if no waypoint, then it will stop and wait
    void start_end_scan (coords_t *start, coords_t *end, int8_t maze[][MAX]) {
        int i, d;
        for (i = 0; i < MAX; i++) {
            for (d = 0; d < MAX; d++) {
                if (maze[i][d] == -1) {
                    start-> x = d;
                    start-> y = i;
                }
                if (maze[i][d] == -2) {
                    end-> x = d;
                    end-> y = i;
                }
            }
        }
        start->g_cost = 0;
        start->h_cost = cartesian_count(start->x, start->y, end->x, end->y);
        end->h_cost = 0;
        end->g_cost = cartesian_count(start->x, start->y, end->x, end->y);
    }
public:
    int probability = 4;
    
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
            {5,0,0,0,0,5,0,5,0,0,5},
            {5,5,0,5,0,0,0,5,0,0,5},
            {5,0,0,0,0,0,0,0,0,0,-1},
            {-2,0,0,0,0,5,5,0,0,0,5},
            {5,0,0,0,0,0,0,0,0,0,5},
            {5,0,0,0,0,0,0,0,0,0,5},
            {5,0,0,0,0,0,0,0,0,0,5},
            {5,0,0,0,0,0,0,0,0,0,5},
            {5,0,0,0,0,0,0,0,0,0,5},
            {5,5,5,5,5,5,5,5,5,5,5}};
        
        
        coords_t *start = new coords_t;
        coords_t *end = new coords_t;
        // We can disable scan_maze
        
        cout << "Original Maze:" << endl;
        print_maze(maze);
        start_end_scan(start, end, maze);
        
        // Initial run
        scan_neighbours (maze, start, *start, *end);
        pathfinder(*start, *end, maze);
        
        // Traceback
        maze[end->y][end->x] = 9; // We use 9 to signify the path
        coords_t* current_point;
        // Search for end point (find end node)
        for (auto iterator = closed.begin(); iterator != closed.end(); ++iterator) {
            if ((*iterator)->x == end->x && (*iterator)->y == end->y) {
                current_point = *iterator;
                break;
            }
        }
        // Iterate thru the next few
        while (current_point->x != start->x || current_point->y != start->y) {
            // Continue tracing
            current_point = current_point->parent;
            maze[current_point->y][current_point->x] = 9;
        }
        cout << "Shortest path:" << endl << endl;
        print_maze (maze);

    }
    
};


int main(int argc, const char * argv[]) {
    A_star map;
    map.generate_path();
    
    return 0;
}

// TODO: Set it up such that it can receive a dynamic map from hector slam and process it then output the shortest path based on a fixed destination on the map



