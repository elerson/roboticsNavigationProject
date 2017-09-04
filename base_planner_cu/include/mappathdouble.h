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
 */

// path structure for use with fdsgraph.h and fdsgraph.c
struct MapPath;
typedef struct MapPath MapPath;

struct MapPath
{  
  double* x;
  double* y;
  int length;
  int storage_length;
};

// creates a new MapPath of length len
MapPath* newMapPath(int len);

// assuming a trail of bestNeighbors links nodeA to nodeB in the graph,
// this puts that path into a new path structure
MapPath* retrievePath(node* nodeA, node* nodeB);

// prints a path to the command line
void printPath(MapPath* path);

// adds the coordinate (x,y) to the end of the map path, returns the result
MapPath* addPathEnd(MapPath* path, double y, double x);

// removes all but the first point in a sub-path containing repeated points
MapPath* removeRepeatedPoints(MapPath* path);

// calculates the cost of traversing the path
double calculatePathCost(MapPath* path);

// deletes a MapPath
void deleteMapPath(MapPath* path);

// this stores only 3 points, which is all that is required to store the 
// path from the robot's current position to the edge of its current cell
struct CellPath;
typedef struct CellPath CellPath;

struct CellPath
{  
  int cell_x;
  int cell_y;
  double x[3];
  double y[3];
  int length;
  double g;
  double local_g;
  double g_to_edge;
  int inHeap;
  int heapIndex;
};

// creates a new CellPath of length len
CellPath* newCellPath(int cell_y, int cell_x);

// copies subPath into a new CellPath and returns a pointer to it
CellPath* copyCellPath(CellPath* subPath);

// prints the info form the cellPath
void printCellPath(CellPath* subPath);

// deletes a CellPath
void deleteCellPath(CellPath* subPath);
