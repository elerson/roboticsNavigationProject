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

// sets up the memory for the heap and retuns a pointer to it
cpHeap* cpBuildHeap()
{
   cpHeap*  thisHeap;
  thisHeap = (cpHeap*)calloc(CPHEAPLENGTH, sizeof(thisHeap));
  thisHeap->cpHeapNode = (CellPath**)calloc(CPHEAPLENGTH, sizeof(CellPath*));
  int i;
  for( i = 0; i < CPHEAPLENGTH; i++)
  {
    thisHeap->cpHeapNode[i] = NULL;  
  }
  thisHeap->cpIndexOfLast = -1;
  thisHeap->cpParentOfLast = -1;
  thisHeap->cpTempInd = -1;
  thisHeap->cpTempNode = NULL;
  return thisHeap;
}

// re-initializes the heap
void cpCleanHeap(cpHeap* thisHeap)
{
  //printf("clearing cellPath heap \n");
  int i;
  for( i = 0; i < CPHEAPLENGTH; i++)
  {
    if(thisHeap->cpHeapNode[i] != NULL)  
    {  
      free(thisHeap->cpHeapNode[i]);
      thisHeap->cpHeapNode[i] = NULL;
    }
  }
  thisHeap->cpIndexOfLast = -1;
  thisHeap->cpParentOfLast = -1;
  thisHeap->cpTempInd = -1;
  thisHeap->cpTempNode = NULL;
}

// compares a node n with its parent, and switches them if the parent's
// cost is more than the node's cost. Repeats if a switch happens.
void cpBubbleUp(cpHeap* thisHeap, int n)
{
  thisHeap->cpTempInd = (n-1)/2;
  while(n != 0 && (thisHeap->cpHeapNode[n]->g < thisHeap->cpHeapNode[thisHeap->cpTempInd]->g))
  {

     // swap graph node pointers
     thisHeap->cpTempNode = thisHeap->cpHeapNode[thisHeap->cpTempInd];
     thisHeap->cpHeapNode[thisHeap->cpTempInd] = thisHeap->cpHeapNode[n];
     thisHeap->cpHeapNode[n] = thisHeap->cpTempNode;

     // update graph node heap index values
     thisHeap->cpHeapNode[thisHeap->cpTempInd]->heapIndex = thisHeap->cpTempInd;
     thisHeap->cpHeapNode[n]->heapIndex = n;

     // get new node and parent indices
     n = thisHeap->cpTempInd;
     thisHeap->cpTempInd = (n-1)/2;
  }   
}

// compares a node n with its children, and switches them if a child's cost
// is less than the node's cost. Repeats if a switch happens.
void cpBubbleDown(cpHeap* thisHeap, int n)
{    
  // find child with smallest value
  if(n > thisHeap->cpParentOfLast)
    return;
  if(2*n+2 > thisHeap->cpIndexOfLast || (thisHeap->cpHeapNode[2*n+1]->g < thisHeap->cpHeapNode[2*n+2]->g))
    thisHeap->cpTempInd = 2*n+1;
  else
    thisHeap->cpTempInd = 2*n+2; 
  
  while(n <= thisHeap->cpParentOfLast && (thisHeap->cpHeapNode[thisHeap->cpTempInd]->g < thisHeap->cpHeapNode[n]->g))
  {  
     // swap graph node pointers
     thisHeap->cpTempNode = thisHeap->cpHeapNode[thisHeap->cpTempInd];
     thisHeap->cpHeapNode[thisHeap->cpTempInd] = thisHeap->cpHeapNode[n];
     thisHeap->cpHeapNode[n] = thisHeap->cpTempNode;  
      
     // update graph node heap index values
     thisHeap->cpHeapNode[thisHeap->cpTempInd]->heapIndex = thisHeap->cpTempInd;
     thisHeap->cpHeapNode[n]->heapIndex = n;
     
     // get new node and child indices
     n = thisHeap->cpTempInd;
     
     if(n > thisHeap->cpParentOfLast)
       return;
     
     if(2*n+2 > thisHeap->cpIndexOfLast || (thisHeap->cpHeapNode[2*n+1]->g < thisHeap->cpHeapNode[2*n+2]->g))
       thisHeap->cpTempInd = 2*n+1;
     else
       thisHeap->cpTempInd = 2*n+2;  
  }   
}

// add thisNode to the heap
void cpAddToHeap(cpHeap* thisHeap, CellPath* thisNode)
{   
  if(thisNode->inHeap == false || thisNode->inHeap == closed)
  {
    thisHeap->cpIndexOfLast = thisHeap->cpIndexOfLast + 1;

    if(thisHeap->cpIndexOfLast == 0)
      thisHeap->cpParentOfLast = -1;
    else
      thisHeap->cpParentOfLast = (thisHeap->cpIndexOfLast-1)/2;
     
    thisHeap->cpHeapNode[thisHeap->cpIndexOfLast] = thisNode;
    thisNode->heapIndex = thisHeap->cpIndexOfLast;
    cpBubbleUp(thisHeap, thisHeap->cpIndexOfLast);
    thisNode->inHeap = true;
  }
}

// returns a pointer to the node that is on the top of the heap
CellPath* cpTopHeap(cpHeap* thisHeap)
{
  return thisHeap->cpHeapNode[0];
}


// removes the top valued node from the heap and returns a pointer to it
CellPath* cpPopHeap(cpHeap* thisHeap)
{
  CellPath* oldTopNode = thisHeap->cpHeapNode[0];
  thisHeap->cpHeapNode[0] = thisHeap->cpHeapNode[thisHeap->cpIndexOfLast];
  thisHeap->cpHeapNode[0]->heapIndex = 0;
  thisHeap->cpHeapNode[thisHeap->cpIndexOfLast] = NULL;
  thisHeap->cpIndexOfLast = thisHeap->cpIndexOfLast - 1;

  if(thisHeap->cpIndexOfLast == 0)
    thisHeap->cpParentOfLast = -1;
  else
    thisHeap->cpParentOfLast = (thisHeap->cpIndexOfLast-1)/2;
  
  cpBubbleDown(thisHeap, 0);
  oldTopNode->inHeap = closed;
  oldTopNode->heapIndex = -1;
  return oldTopNode;
}

// prints the heap values on the command line
void cpPrintHeap(cpHeap* thisHeap)
{
  int i,p; 
  printf("Heap: \n");
  for(i = 0, p = 0; i <= thisHeap->cpIndexOfLast; i++)
  {    
    printf("(%4.8f) ", thisHeap->cpHeapNode[i]->g);
    if(i == p)
    {
       printf("\n");
       p = p+(2^(p));
    }
  }
  printf("\n\n");
}


// returns 1 if heap is good, 0 if bad, also prints a command line message
int cpCheckHeap(cpHeap* thisHeap)
{
  int i;
  for(i = 0; i <= thisHeap->cpIndexOfLast; i++)
  {
    if((thisHeap->cpHeapNode[i]->g < thisHeap->cpHeapNode[(i-1)/2]->g) || thisHeap->cpHeapNode[i]->heapIndex != i)
    {
      printf("There is a problem with the cellPath heap:\n");
      
      if((thisHeap->cpHeapNode[i]->g < thisHeap->cpHeapNode[(i-1)/2]->g))
        printf("node %d is not less than node %d \n", thisHeap->cpHeapNode[i], thisHeap->cpHeapNode[(i-1)/2]);  
      else
        printf("node (%d, %d) thinks it is at index %d instead of %d \n", thisHeap->cpHeapNode[i]->y, thisHeap->cpHeapNode[i]->x, thisHeap->cpHeapNode[i]->heapIndex, i);  
      
      getchar();
      return false;
    }
  } 
  printf("The heap is OK \n");
  return true;  
}

// cleans up and deletes the memory used by the heap
void cpDeleteHeap(cpHeap* thisHeap)
{  
  int i;
  for( i = 0; i < CPHEAPLENGTH; i++)
  {
    if(thisHeap->cpHeapNode[i] != NULL)  
    {  
      free(thisHeap->cpHeapNode[i]);
      thisHeap->cpHeapNode[i] = NULL;
    }
  }
  free(thisHeap->cpHeapNode);
  free(thisHeap);
}
