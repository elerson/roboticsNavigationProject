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
 *  field d* binary heap implementation for use with graph-search on
 *  graphs/maps built with fdsgraph.h and fdsgraph.c. This implements the 
 *  function headers from fdsheap.h.
 */

// sets up the memory for the heap
void buildHeap()
{
  heapNode = (node**)calloc((HEIGHT+1)*(WIDTH+1), sizeof(node*));
  int i;
  for( i = 0; i < (HEIGHT+1)*(WIDTH+1); i++)
    heapNode[i] = NULL;  
  indexOfLast = -1;
  parentOfLast = -1;
  tempNode = NULL;
}

// compares a node n with its parent, and switches them if the parent's
// cost is more than the node's cost. Repeats if a switch happens.
void bubbleUp(int n)
{
  tempInd = (n-1)/2;
  while(n != 0 && nodeLess(heapNode[n], heapNode[tempInd]))
  {
     // swap graph node pointers
     tempNode = heapNode[tempInd];
     heapNode[tempInd] = heapNode[n];
     heapNode[n] = tempNode;
     
     // update graph node heap index values
     heapNode[tempInd]->heapIndex = tempInd;
     heapNode[n]->heapIndex = n;
     
     // get new node and parent indicies
     n = tempInd;
     tempInd = (n-1)/2;
  }   
}

// compares a node n with its children, and switches them if a child's cost
// is less than the node's cost. Repeats if a switch happens.
void bubbleDown(int n)
{    
  // find child with smallest value
  if(n > parentOfLast)
    return;
  if(2*n+2 > indexOfLast || nodeLess(heapNode[2*n+1], heapNode[2*n+2]))
    tempInd = 2*n+1;
  else
    tempInd = 2*n+2; 
  
  while(n <= parentOfLast && nodeLess(heapNode[tempInd], heapNode[n]))
  {  
     // swap graph node pointers
     tempNode = heapNode[tempInd];
     heapNode[tempInd] = heapNode[n];
     heapNode[n] = tempNode;  
      
     // update graph node heap index values
     heapNode[tempInd]->heapIndex = tempInd;
     heapNode[n]->heapIndex = n;
     
     // get new node and child indicies
     n = tempInd;
     
     if(n > parentOfLast)
       return;
     
     if(2*n+2 > indexOfLast || nodeLess(heapNode[2*n+1], heapNode[2*n+2]))
       tempInd = 2*n+1;
     else
       tempInd = 2*n+2;  
  }   
}

// add thisNode to the heap
void addToHeap(node* thisNode)
{ 
  if(thisNode->inHeap == false || thisNode->inHeap == closed)
  {
    indexOfLast++;
    
    if(indexOfLast == 0)
      parentOfLast = -1;
    else
      parentOfLast = (indexOfLast-1)/2;

    heapNode[indexOfLast] = thisNode;
    thisNode->heapIndex = indexOfLast;
    bubbleUp(indexOfLast);     
    thisNode->inHeap = true;
  }
}

// returns a pointer to the node that is on the top of the heap
node* topHeap()
{
  return heapNode[0];
}


// removes the top valued node from the heap and returns a pointer to it
node* popHeap()
{
  node* oldTopNode = heapNode[0];
  heapNode[0] = heapNode[indexOfLast];
  heapNode[0]->heapIndex = 0;
  heapNode[indexOfLast] = NULL;
  indexOfLast--;

  if(indexOfLast == 0)
    parentOfLast = -1;
  else
    parentOfLast = (indexOfLast-1)/2;
  
  bubbleDown(0);
  oldTopNode->inHeap = closed;
  oldTopNode->heapIndex = -1;
  return oldTopNode;
}

// deletes this_node from the heap, and then repairs the heap
void deleteNodeFromHeap(node* this_node)
{ 
  this_node->inHeap = closed;
  tempInd = this_node->heapIndex;  
  this_node->heapIndex = -1;
  
  // if this was the last node on the heap, then we are done
  if(tempInd == indexOfLast)
  {
    indexOfLast--;
    
    if(indexOfLast == 0)
      parentOfLast = -1;
    else
      parentOfLast = (indexOfLast-1)/2;
    
    return;
  }
  else if(tempInd == 0) // if this is the top node, then just pop it
  {
    popHeap();
    return;
  }  
  
  // put last node from heap where this node used to be
  heapNode[tempInd] = heapNode[indexOfLast];
  heapNode[tempInd]->heapIndex = tempInd;
  
  // take care of heap values at old last node position
  heapNode[indexOfLast] = NULL;
  indexOfLast--;
  
  if(indexOfLast == 0)
    parentOfLast = -1;
  else
    parentOfLast = (indexOfLast-1)/2;
  
  // need to repair heap differently depending on the key vector of heapNode[tempInd]
  if(tempInd > parentOfLast) // then can't go down any more so only try to go up
  {
    bubbleUp(tempInd);
  }
  else if(nodeLess(heapNode[tempInd], heapNode[(tempInd-1)/2])) // this node is less than its parent so try to go up
  {
      bubbleUp(tempInd);
  }
  else // this node is not less than its parent, so try to go down
  {
    bubbleDown(tempInd); 
  }
}

// repairs the heap if this_node is in the wrong place in the heap
void updateHeapPositionOfNode(node* this_node)
{ 
  tempInd = this_node->heapIndex;  
  
  // need to repair heap differently depending on the key vector of heapNode[tempInd]
  if(tempInd > parentOfLast) // then can't go down any more so only try to go up
    bubbleUp(tempInd);
  else if(tempInd == 0) // can't go up any more so only try to go down
    bubbleDown(tempInd);
  else if(nodeLess(heapNode[tempInd], heapNode[(tempInd-1)/2])) // this node is less than its parent so try to go up
    bubbleUp(tempInd);  
  else // this node is not less than its parent, so try to go down
    bubbleDown(tempInd);
}

// prints the heap values on the command line
void printHeap()
{
  int i,p; 
  printf("Heap: \n");
  for(i = 0, p = 0; i <= indexOfLast; i++)
  {    
    printf("[%4.1f, %4.1f]", heapNode[i]->k[0], heapNode[i]->k[1]);
    if(i == p)
    {
       printf("\n");
       p = p+(2^(p));
    }
  }
  printf("\n\n");
  
  for(i = 0, p = 0; i <= indexOfLast; i++)
  {    
    printf("(%d, %d) ", heapNode[i]->y,heapNode[i]->x);
    if(i == p)
    {
       printf("\n");
       p = p+(2^(p));
    }
  }
  printf("\n\n");
}


// returns 1 if heap is good, 0 if bad, also prints a command line message
int checkHeap()
{
  int i;
  for(i = 0; i <= indexOfLast; i++)
  {
    if(nodeLess(heapNode[i], heapNode[(i-1)/2]) || heapNode[i]->heapIndex != i)
    {
      printf("There is a problem with the heap:\n");
      
      if(nodeLess(heapNode[i], heapNode[(i-1)/2]))
        printf("node %d is not less than node %d \n", heapNode[i], heapNode[(i-1)/2]);  
      else
        printf("node (%d, %d) thinks it is at index %d instead of %d \n", heapNode[i]->y, heapNode[i]->x, heapNode[i]->heapIndex, i);  
      
      getchar();
      return false;
    }
  } 
  printf("The heap is OK \n");
  return true;  
}

// cleans up and deletes the memory used by the heap
void deleteHeap()
{
  free(heapNode);
  heapNode = NULL;
  indexOfLast = -1;
  parentOfLast = -1;
  tempInd = -1;
  tempNode = NULL;
}
