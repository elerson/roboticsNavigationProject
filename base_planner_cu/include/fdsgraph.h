/*  Copyright Michael Otte, University of Colorado, 9-22-2008, 2009
 *
 *  This file is part of Base_Planner_CU.
 *
 *  Base_Planner_CU is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Base_Planner_CU is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Base_Planner_CU. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  If you require a different license, contact Michael Otte at
 *  michael.otte@colorado.edu
 *
 *  field D* graph search graph and separate map for an N-graph structure 
 */

#ifndef NEIGHBORS
  #define NEIGHBORS 8               // this is the N in N-graph, 4 or 8
  static int dx[] = {1, 0, -1,  0,1,-1,1,-1};
  static int dy[] = {0, 1,  0, -1,1,-1,-1,1};
#endif
 
#ifndef LARGE
  #define LARGE 100000000             // this is the 'infinity' cost
#endif 

#ifndef SMALL
  #define SMALL .000001             // this is the 'epsilon' cost (things below this may be zero)
#endif 

#ifndef false
  #define false 0                   // define false
#endif 

#ifndef true
  #define true 1                    // define true
#endif 

#ifndef closed
  #define closed -1                 // define closed
#endif 

struct node;
typedef struct node node;

struct node
{
    int x, y;                       // this node's coordinates
    
    node* neighbors[NEIGHBORS];     // this holds pointers to neighbors (NULL if unused)
    node* bestNeighbor;             // this is a pointer to the best neighbor (clockwise-most of the two neighbors used for the best edge)
    
    double g;                       // actual cost from start of path to this node
    double h;                       // heuristic estimated cost from this node to end of path
    double f;                       // actual + heuristic cost (g+h)
    double rhs;                     // this is the rhs value for field d* (lite)
    double k[2];                    // this is the key value for field d* (lite)
    
    int inHeap;                     // is this node currently in the heap? (true, false, closed)
    int heapIndex;                  // this stores the heap index of the node
};

node** graph;                // used to create the graph as a 2D array of nodes, global for speed (note: 1 more in Height and Width than map)
double** map;                // used to store the map as a 2D array of doubles, global for speed (note: 1 less in Height and Width than graph)
int HEIGHT;                  // stores the height of the map (height of graph is one more)
int WIDTH;                   // stores the width of the map (width of graph is one more)
node* s_start;               // stores the current node associated with the robot's position
node* s_goal;                // stores the current node associated with the goal position

double mainKey[] = {LARGE, LARGE};           // stores a key that is used in the function main (I did this to save on memory management)
double topKeyKey[] = {LARGE, LARGE};         // stores a key that is returned by the function topKey (I did this to save on memory management)
double calculateKeyKey[] = {LARGE, LARGE};   // stores a key that is returned by the function topKey (I did this to save on memory management)
double computeCostBPKey[] = {0,0,0};         // stores a key that is returned by the function computeCostBP (I did this to save on memory management)

// allocates the initial memory for the graph and map and sets up the graph 
// structure. height and width refer to map, because graph is 1 more. this 
// must be called before any of the next four functions
void buildGraphAndMap(int height, int width);

// creates an N-graph where cost is all 1
void newMapOfOnes();

// creates an N-graph where the costs of percentage density nodes are determined randomly between 1 and maxCost, the rest get values of 1
void newMapOfRandom(double maxCost, double density);

// creates an N-graph where the costs of percentage density nodes are maxCost, the rest get values of 1
void newMapOfBinary(double maxCost, double density);

// creates an N-graph where the costs are determined by fractal map generation, values are between 1 and maxCost, 0 <= bump_factor <= 1 
// 0 <= uniformity <= 1, where closer to 1 means that the maps overall height is more similar, map_type: 0 = normal fractal, 1 = binary, 2 = 3 cost types
void newMapFractal(double maxCost, double bump_factor, double uniformity, int map_type);

// creates an N-graph where the costs are determined by the row major values in the costArray
void newMapFromArray(double* costArray);

// creates an N-graph where the costs are determined by the column major values in the costArray
void newMapFromArrayColumnMajor(double* costArray);

// creates an N-graph where the costs are determined by the bitmap in filename
void newMapFromBitmap(char* filename);

// makes values in the map below (thresh_low thresh_high]
void thresholdMap(double thresh_low, double thresh_high, double val);

// prints the graph on the command line (not too pretty)
void printMap();

// this re-initializes the graph and map, saving cost values
void cleanGraphAndMap();

// this cleans up and deletes the memory associated with the global graph
// and map
void deleteGraphAndMap();

#ifdef CPP
// this cleans up and deletes the memory associated with the global graph
// and map, if delete_map == false, then the map is left intact
void deleteGraphAndMap(bool delete_map);
#endif

// returns true if nodeA < nodeB based on key vectors
int nodeLess(node* nodeA, node* nodeB);

// returns true if nodeA <= nodeB based on key vectors
int nodeLesseq(node* nodeA, node* nodeB);
