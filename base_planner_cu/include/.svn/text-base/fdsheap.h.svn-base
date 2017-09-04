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
 *  basic binary heap implimentation for use with graph-search on
 *  graphs/maps built with fdsgraph.h and fdsgraph.c
 *  
 *  Assumes that the graph has already been built, i.e. the globals
 *  HEIGHT, WIDTH, and LARGE have been initialized. Nonexistant nodes are
 *  initialized to have a cost of LARGE
 */

double* heapCost;     // cost values
node** heapNode;      // pointers to the corresponding map nodes
int indexOfLast;      // the index of the last node in the heap array
int parentOfLast;     // stores the index of the parent of the last node
int tempInd;          // used to help swap nodes
node*  tempNode;      // used to help swap nodes

// sets up the memory for the heap
void buildHeap();

// compares a node n with its parent, and switches them if the parent's
// cost is more than the node's cost. Repeats if a switch happens.
void bubbleUp(int n);

// compares a node n with its children, and switches them if a child's cost
// is less than the node's cost. Repeats if a switch happens.
void bubbleDown(int n);

// add thisNode to the heap
void addToHeap(node* thisNode);

// returns a pointer to the node that is on the top of the heap
node* topHeap();

// removes the top valued node from the heap and returns a pointer to it
node* popHeap();

// deletes this_node from the heap, and then repairs the heap
void deleteNodeFromHeap(node* this_node);

// repairs the heap if this_node is in the wrong place in the heap
void updateHeapPositionOfNode(node* this_node);

// prints the heap values on the command line
void printHeap();

// returns 1 if heap is good, 0 if bad, also prints a command line message
int checkHeap();

// cleans up and deletes the memory used by the heap
void deleteHeap();
