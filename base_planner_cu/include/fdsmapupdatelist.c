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

// initializes an update list
MapUpdateList* newMapUpdateList(int len)
{    
  MapUpdateList* updateList;
  updateList = (MapUpdateList*)calloc(1, sizeof(MapUpdateList));
    
  updateList->length = len;
  updateList->rows = (int*)calloc(len, sizeof(int));  
  updateList->columns = (int*)calloc(len, sizeof(int));  
  updateList->obstacle = (double*)calloc(len, sizeof(double)); 
  updateList->change = (double*)calloc(len, sizeof(double)); 
  return updateList;
}

// returns true if the node [y,x] is in the update list
int inUpdateList(int y, int x, MapUpdateList* updateList)
{
  int i;
  for(i = 0; i < updateList->length; i++)
  {
    if(updateList->columns[i] == x && updateList->rows[i] == y)
      return true;
  }
  return false;  
}

// returns a random update list of length len with values between 1 and maxcost
MapUpdateList* getRandomUpdateList(int len, double maxCost)
{ 
  MapUpdateList* updateList = newMapUpdateList(len);
  
  int i, newx, newy;  
  for (i = 0; i < len; ++i)
  {
    newy = rand() % (HEIGHT);
    newx = rand() % (WIDTH);
    
    while(inUpdateList(newy, newx, updateList) == true)
    {
       newy = rand() % (HEIGHT);
       newx = rand() % (WIDTH); 
    }
    
    updateList->rows[i] = newy;
    updateList->columns[i] = newx;
    updateList->obstacle[i] =  1 + (double)((rand() % (int)(1000*(maxCost-1)+1)))/1000;
    updateList->change[i] = updateList->obstacle[i] - map[updateList->rows[i]][updateList->columns[i]];
  }
  return updateList;
}

// returns a random update list of length len with values between 1 and maxcost
// update positions are in a (square) area centered on map[y][x] of radius radius]
// NOTE this does not do a check to make sure they are within the map
MapUpdateList* getRandomUpdateListNear(int len, double maxCost, int y, int x, int radius)
{
  MapUpdateList* updateList = newMapUpdateList(len);
  
  int i, newx, newy;  
  for (i = 0; i < len; ++i)
  {
    newy = y + (rand() % (radius*2 + 1)) - radius;
    newx = x + (rand() % (radius*2 + 1)) - radius;
    
    if(newy < 0)
      newy = 0;
    else if(newy >= HEIGHT)
      newy = HEIGHT-1;
        
    if(newx < 0)
      newx = 0;
    else if(newx >= WIDTH)
      newx = WIDTH-1; 
    
    while(inUpdateList(newy, newx, updateList) == true)
    {
      newy = y + (rand() % (radius*2 + 1)) - radius;
      newx = x + (rand() % (radius*2 + 1)) - radius;
    
     // if(newy < 0)
     //   newy = 0;
     // else if(newy >= HEIGHT)
     //   newy = HEIGHT-1;
        
     // if(newx < 0)
     //   newx = 0;
     // else if(newx >= WIDTH)
     //   newx = WIDTH-1; 
    }
    
    updateList->rows[i] = newy;
    updateList->columns[i] = newx;
    updateList->obstacle[i] =  1 + (double)((rand() % (int)(1000*(maxCost-1)+1)))/1000;
   // updateList->change[i] = updateList->obstacle[i] - map[updateList->rows[i]][updateList->columns[i]];
  }
  return updateList;
}

// prints the update list to the command line
void printUpdateList(MapUpdateList* updateList)
{
  int i;
  for (i = 0; i < updateList->length; i++)    
    printf("[%d, %d] =  %f \n", updateList->rows[i], updateList->columns[i], updateList->obstacle[i]);
  printf("\n");
}

// updates map obstacle costs (NOT THE HEAP OR THE GRAPH, g, h, or f) with the update list
void updateGraph(MapUpdateList* updateList)
{
  int i;
  for (i = 0; i < updateList->length; i++)
  map[updateList->rows[i]][updateList->columns[i]] = updateList->obstacle[i];
}

// deletes an updatelist
void deleteMapUpdateList(MapUpdateList* updateList)
{
  free(updateList->rows);
  free(updateList->columns);
  free(updateList->obstacle);
  free(updateList->change);
  free(updateList);
}
