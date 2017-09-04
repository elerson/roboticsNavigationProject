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


// update list structure for use with fdsgraph.h and fdsgraph.c
struct MapUpdateList;
typedef struct MapUpdateList MapUpdateList;

struct MapUpdateList
{  
  int* rows;
  int* columns;
  double* obstacle; // this is cost -- M.O.
  double* change; // stores the change in cost from the previous value
  int length;
};

// initializes an update list
MapUpdateList* newMapUpdateList(int len);

// returns true if the node [y,x] is in the update list
int inUpdateList(int y, int x, MapUpdateList* updateList);

// returns a random update list of length len with values between 1 and maxcost
MapUpdateList* getRandomUpdateList(int len, double maxCost);

// returns a random update list of length len with values between 1 and maxcost
// update positions are in a (square) area centered on map[y][x] of radius radius
MapUpdateList* getRandomUpdateListNear(int len, double maxCost, int y, int x, int radius);

// prints the update list to the command line
void printUpdateList(MapUpdateList* updateList);

// updates map obstacle costs (NOT THE HEAP, g, h, or f) with the update list
void updateGraph(MapUpdateList* updateList);

// deletes an updatelist
void deleteMapUpdateList(MapUpdateList* updateList);
