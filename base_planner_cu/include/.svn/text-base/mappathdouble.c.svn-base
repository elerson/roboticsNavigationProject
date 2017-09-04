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

// initializes a MapPath
MapPath* newMapPath(int len)
{    
  MapPath* path;
  path = (MapPath*)calloc(1, sizeof(MapPath));
  path->length = len;
  path->storage_length = len;
  if(len <= 0)
  {
    path->x = NULL;  
    path->y = NULL;    
  }
  else
  {
    path->x = (double*)calloc(len, sizeof(double));  
    path->y = (double*)calloc(len, sizeof(double)); 
  }
  return path;
}

// assuming a trail of bestNeighbors links nodeA to nodeB in the graph,
// this puts that path into a new path structure
MapPath* retrievePath(node* nodeA, node* nodeB)
{
  // find length of path
  node* nodeC = nodeA;
  int i = 1;
  while(nodeC != nodeB)
  {
      //printf("node: %d %d \n", nodeC->y, nodeC->x);
      
    if(nodeC->bestNeighbor->bestNeighbor == nodeC && nodeC != s_goal && nodeC->bestNeighbor != s_goal)
    { 
      printf("problem here [%f %f]: %f %f [%f %f]: %f %f\n", nodeC->bestNeighbor->y, nodeC->bestNeighbor->x, nodeC->bestNeighbor->g,nodeC->bestNeighbor->rhs, nodeC->bestNeighbor->bestNeighbor->y, nodeC->bestNeighbor->bestNeighbor->x,nodeC->bestNeighbor->bestNeighbor->g,nodeC->bestNeighbor->bestNeighbor->rhs);    
      //getchar();    
    }
    nodeC = nodeC->bestNeighbor;
    i++;
  }
  
  // make a new path
  MapPath* path = newMapPath(i);  
  
  // put path in the list
  nodeC = nodeA;
  for(i = 0; nodeC != nodeB; i++ , nodeC = nodeC->bestNeighbor)
  {
    path->x[i] = nodeC->x;
    path->y[i] = nodeC->y;
  }
  path->x[path->length-1] = nodeB->x;
  path->y[path->length-1] = nodeB->y;
  return path;
}

// prints a path to the command line
void printPath(MapPath* path)
{
  int i;
  for(i = 0; i < path->length; i++)   
    printf("[%f, %f]\n", path->y[i], path->x[i]);
}

// adds the coordinate (x,y) to the end of the map path, returns the result
MapPath* addPathEnd(MapPath* path, double y, double x)
{
  if(path->length == path->storage_length)
  {  
    MapPath* pathNew = newMapPath(4+path->length*2);
    int i;
    for(i = 0; i < path->length; i++)
    {
      pathNew->x[i] = path->x[i]; 
      pathNew->y[i] = path->y[i];
    }
    pathNew->x[path->length] = x; 
    pathNew->y[path->length] = y;  
    pathNew->length = path->length + 1;
    
    deleteMapPath(path);
    return pathNew;
  }
  else
  {
    path->x[path->length] = x; 
    path->y[path->length] = y;  
    path->length = path->length + 1;
    return path;
  }
}

// removes all but the first point in a sub-path containing repeated points
MapPath* removeRepeatedPoints(MapPath* path)
{
  // walk through once to figure out the size of the new path
  int i,j;
  int sizeOfNew = 1;
  int lastUnique = 0;
  
  for(i = 1; i < path->length; i++)
  {
    if(path->x[lastUnique] != path->x[i] || path->y[lastUnique] != path->y[i]) // i is the next unique point
    {
      sizeOfNew = sizeOfNew + 1;   
      lastUnique = i;  
    }
  }

  // create new path  
  MapPath* pathNew = newMapPath(sizeOfNew);
  
  // populate new path
  pathNew->x[0] = path->x[0]; 
  pathNew->y[0] = path->y[0];

  j = 0;
  for(i = 1; i < path->length; i++)
  {
    if(pathNew->x[j] != path->x[i] || pathNew->y[j] != path->y[i]) // i is the next unique point
    {
      j++;
      pathNew->x[j] = path->x[i]; 
      pathNew->y[j] = path->y[i];
    }
  }
  
  deleteMapPath(path);
    
  return pathNew;
}

// returns the lesser of two values
double double_min(double a, double b)
{
  if(a < b)
   return a;
  return b;
}

// returns the absolute value of a value
double double_abs(double a)
{
  if(a > 0)
   return a;
  return -a;
}

// calculates the cost of traversing the path
double calculatePathCost(MapPath* path)
{
  double total_cost = 0;
  double new_cost;
  int i,n,m;

  for(i = 1; i < path->length; i++)
  {
    if(path->x[i-1] == path->x[i] && path->x[i] == (double)((int)(path->x[i]))) // path segment is on a vertical edge 
    {
      n = (int)double_min(path->y[i],path->y[i-1]); // this is the row of the map grid

      // the edge is between map[n][path->x[i]-1] and map[][path->x[i], we want to use the least cost of the two
      if(path->x[i]-1 < 0) // then on edge of map
        new_cost = map[n][(int)(path->x[i])];
      else if((int)(path->x[i]) == WIDTH) // on other edge of map
        new_cost = map[n][(int)(path->x[i]-1)];
      else if(map[n][(int)(path->x[i]-1)] < map[n][(int)(path->x[i])])
        new_cost = map[n][(int)(path->x[i]-1)];
      else
        new_cost = map[n][(int)(path->x[i])];

      total_cost = total_cost + new_cost*(double_abs(path->y[i]-path->y[i-1]));
    } 
    else if(path->y[i-1] == path->y[i] && path->y[i] == (double)((int)(path->y[i]))) // path segment is on a horizontal edge
    {
      m = (int)double_min(path->x[i],path->x[i-1]); // this is the column of the map grid

      // the edge is between map[path->y[i]-1][m] and map[path->y[i][m], we want to use the least cost of the two
      if(path->y[i]-1 < 0) // then on edge of map
        new_cost = map[(int)(path->y[i])][m];
      else if((int)(path->y[i]) == HEIGHT) // on other edge of map
        new_cost = map[(int)(path->y[i]-1)][m];
      else if(map[(int)(path->y[i]-1)][m] < map[(int)(path->y[i])][m])
        new_cost = map[(int)(path->y[i]-1)][m];
      else
        new_cost = map[(int)(path->y[i])][m];

      total_cost = total_cost + new_cost*(double_abs(path->x[i]-path->x[i-1]));
    }
    else // path segment goes through the grid map[floor(min(path->y[i],path->y[i-1]))] [floor(min(path->x[i],path->x[i-1]))]
    {
      new_cost = (sqrt( ((path->y[i]-path->y[i-1])*(path->y[i]-path->y[i-1])) + ((path->x[i]-path->x[i-1])*(path->x[i]-path->x[i-1])))) * map[(int)double_min(path->y[i],path->y[i-1])][(int)double_min(path->x[i],path->x[i-1])];

      total_cost = total_cost + new_cost;
    }
  }
  return total_cost;
}

// deletes a MapPath
void deleteMapPath(MapPath* path)
{
  if(path->length > 0)
  {
     free(path->x);
     free(path->y);   
  }
  free(path);
}

// creates a new CellPath of length len
CellPath* newCellPath(int cell_y, int cell_x)
{
  CellPath* subPath = (CellPath*)calloc(1, sizeof(CellPath)); 
  subPath->cell_y = cell_y;
  subPath->cell_x = cell_x;
  subPath->length = 0;
  subPath->inHeap = false;
  return subPath;
}

// copies subPath into a new CellPath and returns a pointer to it
CellPath* copyCellPath(CellPath* subPath)
{
  int i;
  CellPath* newPath = (CellPath*)calloc(1, sizeof(CellPath));  
    
  newPath->cell_x = subPath->cell_x;
  newPath->cell_y = subPath->cell_y;
  
  for(i = 0; i < 3; i++)
  {
    newPath->x[i] = subPath->x[i];
    newPath->y[i] = subPath->y[i];
  }
  
  newPath->length = subPath->length;
  newPath->g = subPath->g;
  newPath->local_g = subPath->local_g;
  newPath->g_to_edge = subPath->g_to_edge;
  newPath->inHeap = subPath->inHeap;
  newPath->heapIndex = subPath->heapIndex;  
  return newPath;
}

// prints the info form the cellPath
void printCellPath(CellPath* subPath)
{  
  int i;
  printf("cell_x: %d \n", subPath->cell_x);
  printf("cell_y: %d \n", subPath->cell_y);
  
  printf("x[]: ");
  for(i = 0; i < subPath->length; i++)
    printf("%f, ", subPath->x[i]);
  printf("\ny[]: ");
  for(i = 0; i < subPath->length; i++)
    printf("%f, ", subPath->y[i]);
  printf("\n");
  
  
  printf("length: %d \n", subPath->length);
  printf("g: %f \n", subPath->g);
  printf("local_g: %f \n", subPath->local_g);
  printf("g_to_edge: %f \n", subPath->g_to_edge);
  printf("inHeap: %d \n", subPath->inHeap);
  printf("heapIndex: %d \n\n", subPath->heapIndex);
}


// deletes a CellPath
void deleteCellPath(CellPath* subPath)
{
  free(subPath);
}
