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
 *  basic graph display kit for graphs built with graph.h and graph.c
 *  assumes that glut, glu, and gl are included in the path
 */

// returns the max of two values
double max(double a, double b)
{
  if(a > b)
      return a;
  else
      return b;
}

// returns the min of two values
double min(double a, double b)
{
  if(a < b)
      return a;
  else
      return b;
}

// takes values ranging from 1 to mult and scales to 1 to 0
double scaleColor(double col, double mult)
{
  return 1 - max(0,min((col-1)/(mult-.99999),1));  
}

#ifndef NOGLUT

// draws a rectangle
void drawRectangle(double* pos, double* size, double* color)
{
  glPushMatrix();
    glTranslatef(pos[0],pos[1],pos[2]);
    glScaled(size[0],size[1],1); 
   
    glBegin(GL_POLYGON);
      glColor3f(color[0],color[1],color[2]); 
        glVertex2f(0,1);
        glVertex2f(1,1);
        glVertex2f(1,0);
        glVertex2f(0,0);
    glEnd(); 
   
  glPopMatrix();
}

// draws a rectangle with a different color on each corner
void drawRectangleMultiColor(double* pos, double* size, double* color1, double* color2, double* color3, double* color4)
{
  glPushMatrix();
    glTranslatef(pos[0],pos[1],pos[2]);
    glScaled(size[0],size[1],1); 
   
    glBegin(GL_POLYGON);
      glColor3f(color1[0],color1[1],color1[2]); 
        glVertex2f(0,1);
      glColor3f(color2[0],color2[1],color2[2]);   
        glVertex2f(0,0);
      glColor3f(color3[0],color3[1],color3[2]);     
        glVertex2f(1,0);
      glColor3f(color4[0],color4[1],color4[2]);   
        glVertex2f(1,1);
    glEnd(); 
   
  glPopMatrix();
}

// draws the current map
// max_cost of -1 takes max cost to be the maximum map value
void drawMap(double max_cost, double z_height)
{  
  int r,c;
  double x;
  double y = 1;
    
  double grid_height = 2/((double)HEIGHT);
  double grid_width = 2/((double)WIDTH);
  
  int green_flag = 0; // 1 = draw in green scale, 0 = greyscale

  if(max_cost == -1) // need to find max map cost 
    for (r = 0; r < HEIGHT; r++)    
	  for (c = 0; c < WIDTH; c++)
	    if (map[r][c] > max_cost)
          max_cost = map[r][c];
  
  for (r = 0; r < HEIGHT; r++)
  {    
    y = y - grid_height; 
    x = -1 - grid_width;
      
	for (c = 0; c < WIDTH; c++)
	{
       x = x + grid_width; 
        
       double loc[] = {x,y,z_height};
       double siz[] = {grid_width,grid_height};

       double clr[] = {0,scaleColor(map[r][c],max_cost),0};
       if(green_flag == 0)
       {
         clr[0] = scaleColor(map[r][c],max_cost);
         clr[2] = scaleColor(map[r][c],max_cost);
       }

       drawRectangle(loc, siz, clr); 
	}
  }  
}

// draws the current field based on the g values of the graph
void drawField(double max_g_value, double z_height)
{  
  int r,c;
  double grid_height = 2/((double)HEIGHT);
  double grid_width = 2/((double)WIDTH);
  double x;
  double y = 1;
  
  if(max_g_value == -1) // need to find max map cost 
    for (r = 0; r < HEIGHT+1; r++)    
	  for (c = 0; c < WIDTH+1; c++)
	    if (graph[r][c].g > max_g_value && graph[r][c].g < LARGE)
          max_g_value = graph[r][c].g;
  
  for (r = 0; r < HEIGHT; r++)
  {    
    y = y - grid_height; 
    x = -1 - grid_height;
      
	for (c = 0; c < WIDTH; c++)
	{
       x = x + grid_width; 
       
       double siz[] = {grid_width,grid_height};
       double loc[] = {x,y,z_height};
       
       double clr1[] = {0,scaleColor(graph[r][c].g,max_g_value),0};
       if(clr1[1] == 0)
       {
         clr1[0] = 1;  
         clr1[1] = 1;  
         clr1[2] = 1;   
       }
       
       double clr2[] = {0,scaleColor(graph[r+1][c].g,max_g_value),0};
       if(clr2[1] == 0)
       {
         clr2[0] = 1;
         clr2[1] = 1;
         clr2[2] = 1;   
       }
       
       double clr3[] = {0,scaleColor(graph[r+1][c+1].g,max_g_value),0};
       if(clr3[1] == 0)
       {
         clr3[0] = 1;  
         clr3[1] = 1;  
         clr3[2] = 1;   
       }

       double clr4[] = {0,scaleColor(graph[r][c+1].g,max_g_value),0};
       if(clr4[1] == 0)
       {
         clr4[0] = 1; 
         clr4[1] = 1; 
         clr4[2] = 1;   
       }
       drawRectangleMultiColor(loc, siz, clr1, clr2, clr3, clr4);
     }
  }
}

// draws the current heap (open, closed, closed and no neighbors back-link) 
// on top of the graph
void drawHeapOnMaze(double z_height)
{  
  int r,c,n;    
  double grid_height = 2/((double)HEIGHT);
  double grid_width = 2/((double)WIDTH);
  double x;
  double y = 1+grid_height/3+grid_height/2;
  
  for (r = 0; r < HEIGHT+1; r++)
  {    
    y = y - grid_height; 
    x = -1 - 2*grid_width/3-grid_height/2;
      
	for (c = 0; c < WIDTH+1; c++)
	{
       x = x + grid_width; 
        
       double loc[] = {x,y,z_height};
       double siz[] = {grid_width/3,grid_height/3};
       if(graph[r][c].inHeap == true)
       {
         double clr[] = {1,0,0};  
         drawRectangle(loc, siz, clr); 
       }
       else if(graph[r][c].inHeap == closed)
       {  
         double clr[] = {1,1,0};  // yellow
   
         for(n = 0; n < NEIGHBORS; n++)
         {
           if(graph[r][c].neighbors[n] != NULL)
           {
             if(graph[r][c].neighbors[n]->bestNeighbor != NULL)   
             {
               if(graph[r][c].neighbors[n]->bestNeighbor->x == graph[r][c].x && graph[r][c].neighbors[n]->bestNeighbor->y == graph[r][c].y) 
               {
                 // blue
                 clr[0] = 0; 
                 clr[1] = 0; 
                 clr[2] = 1;
               }
             }
           }
         }
         drawRectangle(loc, siz, clr);   
       }
     }
  }
}
 
// draws a path on the maze at height z_height
void drawPathOnMaze(MapPath* path, double* color, double z_height)
{
  if(path->length <= 0)
    return;
  glPushMatrix();
  
  glTranslatef(-1,1,z_height);
  glScaled(2/((double)WIDTH),-2/((double)HEIGHT),1); 
    
  glColor3f(color[0],color[1],color[2]);
  glBegin(GL_LINES);
    int i;
    glVertex3d((double)path->x[0],(double)path->y[0],0);
    for (i = 1; i < path->length; i++)
    {
        
      glVertex3d((double)path->x[i],(double)path->y[i],0);
      glVertex3d((double)path->x[i],(double)path->y[i],0);
    }
    glVertex3d((double)path->x[path->length-1],(double)path->y[path->length-1],0);
  glEnd();
  glPopMatrix();
}


// draws squares around grids that are in the update list at height z_height
void drawUpdateList(MapUpdateList* updateList, double z_height)
{  
  glPushMatrix();
  
  glTranslatef(-1+1/((double)WIDTH),1-1/((double)HEIGHT),z_height);
  glScaled(2/((double)WIDTH),-2/((double)HEIGHT),1); 
 
  glBegin(GL_LINES);
  int i;
  for(i = 0; i < updateList->length; i++)
  {
      
    if(updateList->change[i] > 0)
      glColor3f(1,0,0);  
    else
      glColor3f(0,0,1);   
        
    glVertex3d((double)updateList->columns[i]-.5, (double)updateList->rows[i]-.5,0);   
    glVertex3d((double)updateList->columns[i]-.5, (double)updateList->rows[i]+.5,0); 
    
    glVertex3d((double)updateList->columns[i]-.5, (double)updateList->rows[i]+.5,0); 
    glVertex3d((double)updateList->columns[i]+.5, (double)updateList->rows[i]+.5,0); 
    
    glVertex3d((double)updateList->columns[i]+.5, (double)updateList->rows[i]+.5,0); 
    glVertex3d((double)updateList->columns[i]+.5, (double)updateList->rows[i]-.5,0); 
    
    glVertex3d((double)updateList->columns[i]+.5, (double)updateList->rows[i]-.5,0); 
    glVertex3d((double)updateList->columns[i]-.5, (double)updateList->rows[i]-.5,0);  
  } 
  glEnd();
  glPopMatrix();  
}

#endif
