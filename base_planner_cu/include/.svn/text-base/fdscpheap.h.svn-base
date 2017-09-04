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
 *  basic binary heap implementation for use with cellPaths
 *  Nonexistent nodes are initialized to have a cost of LARGE
 */

#ifndef CPHEAPLENGTH
  #define CPHEAPLENGTH 50    // this is the size of the cellPath heap
#endif 

struct cpHeap;
typedef struct cpHeap cpHeap;

struct cpHeap
{
  double* cpHeapCost;       // cost values
  CellPath** cpHeapNode;    // pointers to the corresponding CellPaths
  int cpIndexOfLast;        // the index of the last node in the heap array
  int cpParentOfLast;       // stores the index of the parent of the last node
  int cpTempInd;            // used to help swap nodes
  CellPath*  cpTempNode;    // used to help swap nodes
};

// sets up the memory for the heap and returns a pointer to it
cpHeap* cpBuildHeap();

// re-initializes the heap
void cpCleanHeap(cpHeap* thisHeap);

// compares a node n with its parent, and switches them if the parent's
// cost is more than the node's cost. Repeats if a switch happens.
void cpBubbleUp(cpHeap* thisHeap, int n);

// compares a node n with its children, and switches them if a child's cost
// is less than the node's cost. Repeats if a switch happens.
void cpBubbleDown(cpHeap* thisHeap, int n);

// add thisNode to the heap
void cpAddToHeap(cpHeap* thisHeap, CellPath* thisNode);

// returns a pointer to the node that is on the top of the heap
CellPath* cpTopHeap(cpHeap* thisHeap);

// removes the top valued node from the heap and returns a pointer to it
CellPath* cpPopHeap(cpHeap* thisHeap);

// prints the heap values on the command line
void cpPrintHeap(cpHeap* thisHeap);

// returns 1 if heap is good, 0 if bad, also prints a command line message
int cpCheckHeap(cpHeap* thisHeap);

// cleans up and deletes the memory used by the heap
void cpDeleteHeap(cpHeap* thisHeap);
