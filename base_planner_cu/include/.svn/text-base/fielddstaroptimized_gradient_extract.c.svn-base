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
 *  field D* Algorithm, work based, edge cost = distance*(node1+node2)/2.
 *  Uses gradient based method of extraction instead of 1-step look ahead.
 *  Distance to goal heuristic = Euclidean. this is the optimized version.
 *  This code assumes truncating conversion from double to int
 *
 *  compile as follows: gcc fielddstaroptimized_gradient_extract.c -lglut
 *
 *  I have had some problems with this hanging
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include </usr/include/GL/glut.h>	    // GLUT
#include </usr/include/GL/glu.h>	    // GLU
#include </usr/include/GL/gl.h>	        // OpenGL

#include "fdsgraph.h"
#include "fdsgraph.c"
#include "fdsheap.h"
#include "fdsheap.c"
#include "mappathdouble.h"
#include "mappathdouble.c"
#include "fdsmapupdatelist.h"
#include "fdsmapupdatelist.c"
#include "fdsgraphgraphics.h"
#include "fdsgraphgraphics.c"
#include "fdscpheap.h"
#include "fdscpheap.c"  

#ifndef sqrt2
  #define sqrt2 1.414213562373095   
#endif 

#ifndef PI
  #define PI 3.141592653589793   
#endif 
      
//#define R2GOAL // when defined, this allows the goal to exist anywhere in the map, not just at integer (i,j). Note: this is a hack job and is still buggy with certain types of maps     
        
double clrY[] = {1,1,0}; // yellow
double clrP[] = {1,0,1}; // purple
double clrB[] = {0,0,1}; // blue
double clrW[] = {1,1,1}; // white
double clrR[] = {1,0,0}; // red
double clrK[] = {0,0,0}; // black
double clrG[] = {.5,.5,.5}; // grey

int deltaY[] = {0, 0, 1, 1}; // used to find nodes on the corners of a grid cell
int deltaX[] = {0, 1, 1, 0}; // used to find nodes on the corners of a grid cell
int corners = 4;

double distanceLength[] = {1,1,1,1,sqrt2,sqrt2,sqrt2,sqrt2}; // 8-connected distances
double maxCost = 10; // this is needed for H, and is the max (single cell) map cost possible

#ifndef  NOGLUT
int DISPLAYGRAPHICS = 1; // 0 = no 1 = yes
#endif

double robot_pos_x;  // stores location of robot in field
double robot_pos_y;  // stores location of robot in field

#ifdef R2GOAL // goal can be anywhere in R2
  double goalRN; // stores location of R2 goal
  double goalRM;  // stores location of R2 goal
#endif

cpHeap* primaryCellPathHeap; // stores local cost values during path extraction
cpHeap* secondaryCellPathHeap; // stores local cost values during 1 step look ahead

double current_path_cost; // this is a small hack to get path cost out (used only for visualization)

long this_seed; // holds the random seed

/*------------------------- Start of Graphics stuff -- M.O. -----------------------------------*/
#ifndef  NOGLUT
void clearDisplay()
{
   glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
   glLoadIdentity();
}

void flushDisplay()
{
   glutPostRedisplay();
   glFlush();
   glutSwapBuffers();
}

void reshape(int width,int height)
{
   double w2h = (height>0) ? (double)width/height : 1;
   glViewport(0,0, width,height);
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   glOrtho(-w2h,+w2h, -1.0,+1.0, -1.0,+1.0);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
}
#endif
/*------------------------- End of Graphics stuff -- M.O. -------------------------------------*/


void pauseFor(double sec)
{
  clock_t endtime;
  endtime = clock() + (int)(sec*((double)(CLOCKS_PER_SEC)));
  while (clock() < endtime) 
  {}
}

double myMax(double a, double b)
{
  if( a > b)
      return a;
  return b;
}

// returns the heuristic estimate of the cost from nodeA to nodeB
// in this case the larger horizontal or vertical distance, however
// the max cost of cell traversal is subtracted first
double H(node* nodeA, node* nodeB)
{
  #ifdef R2GOAL
    return 0;
  #else
    //return myMax(sqrt((nodeA->x - nodeB->x)*(nodeA->x - nodeB->x) + 
    //              (nodeA->y - nodeB->y)*(nodeA->y - nodeB->y)) - 2*maxCost*sqrt2,0); // paper says need to - max cost of traversal here 
  
    // when using lethal values, the max cost of traversal is >> than the euclidean distance so just return 0;
    return 0;
  #endif
}

// returns true if keyA < keyB based on the first two array values
int keyLess(double* keyA, double* keyB)
{
  if(keyA[0] < keyB[0])
      return true;
  else if(keyA[0] == keyB[0] && keyA[1] < keyB[1])
      return true;
  return false;
}

// returns true if keyA[0] < keyB[0] or they are equal and keyA[0]<=keyB[0]
int keyLesseq(double* keyA, double* keyB)
{
  if(keyA[0] < keyB[0])
      return true;
  else if(keyA[0] == keyB[0] && keyA[1] <= keyB[1])
      return true;
  return false;
}

// This is function U.TopKey from the paper, it returns the key of the top
// node on the heap, I am using the global topKeyKey to store the returned 
// key for coding ease.
double* topKey()
{
  node* top = topHeap();

  if(top == NULL) // I added this to fix a case that happens when s_start is the only node in the heap and then it is popped and no new nodes are added
  {
    topKeyKey[0] = LARGE;   // having LARGE here is good for this case because it forces the while loop check to fail in computShortestPath
    topKeyKey[1] = LARGE;  
  }
  else
  {
    topKeyKey[0] = top->k[0];   
    topKeyKey[1] = top->k[1]; 
  }
  
  return topKeyKey;
}

// This is function U.Insert from the paper, it inserts the node into the 
// heap using the new key newKey
void insert(node* s, double* newKey)
{
  s->k[0] = newKey[0];
  s->k[1] = newKey[1];
  addToHeap(s);
}

// This is function U.Update from the paper, it changes the key of the node 
// and then re-orders the heap to account for the new key value
void update(node* s, double* newKey)
{
  s->k[0] = newKey[0];
  s->k[1] = newKey[1];
  updateHeapPositionOfNode(s);
}

// This is procedure Key from the paper, it calculates the key of 
// a node, and then returns the key. I am using the global calculateKeyKey 
// to store the returned key for coding ease.
double* calculateKey(node* s)
{
  calculateKeyKey[0] = min(s->g,s->rhs) + H(s_start,s);
  calculateKeyKey[1] = min(s->g,s->rhs); 
  return calculateKeyKey;
}

// this returns the minimum of the three values
int intMin3(int a, int b, int c)
{
  if(a <= b && a <= c)
      return a;
  else if(b <= c)
      return b;
  return c;
}

// this returns the minimum of the two values
int intMin2(int a, int b)
{
  if(a <= b)
      return a;
  return b;
}

// this is function ComputeCost(s,s_a,s_b) from the paper, it returns the 
// minimum 'g' value for s along the edge connecting s_a and s_b
double computeCost(node* s, node* s_a, node* s_b)
{
  node* s_1;
  node* s_2;
  double c, b, f, v_s, y, x;
  int b_y, b_x;
  
  if(s == NULL || s_a == NULL || s_b == NULL)
    return LARGE*2;

  if(s->x == s_b->x || s->y == s_b->y) // then s_b is horizontal or vertical neighbor of s, thus, s_a is a diagonal neighbor of s
  {
    s_1 = s_b;  
    s_2 = s_a;
  }
  else
  {
    s_1 = s_a;  
    s_2 = s_b;  
  }

  // c is traversal cost of map cell with corners s, s_1, s_2
  c = map[intMin3(s->y, s_1->y, s_2->y)][intMin3(s->x, s_1->x, s_2->x)];

  // b is traversal cost of map cell with corners s and s_1, not s_2
  b_y = intMin2(s->y, s_1->y);
  b_x = intMin2(s->x, s_1->x);

  // the above assumes that either c is above b or c is to the left of b, we need to modify things if this is not the case
  if(s_1->y < s_2->y) // then c is actually below b
    b_y--;
  else if(s_1->x < s_2->x)  // then c is actually to the right of b
    b_x--;
  
  // need to take care of edge cases (WIDTH or HEIGHT or -1 or -1)
  if(b_y == HEIGHT || b_x == WIDTH || b_y == -1 || b_x == -1)
    b = c;
  else
    b = map[b_y][b_x];
      
  if(min(c,b) >= LARGE)
    v_s = LARGE; 
  else if(s_1->g <= s_2->g)
    v_s = min(c,b) + s_1->g;
  else
  {
    f = s_1->g - s_2->g;
    if(f <= b)
    {
      if(c <= f)   
        v_s = c*sqrt2 + s_2->g;
      else
      {
        y = min(f/sqrt(c*c - f*f),1);
        v_s = (c*sqrt(1+y*y)) + (f*(1-y)) + s_2->g;
      }
    }
    else
    {
      if(c <= b)
        v_s = c*sqrt2 + s_2->g;
      else
      {
        x = 1 - min(b/sqrt(c*c - b*b),1);  
        v_s = (c*sqrt(1+((1-x)*(1-x)))) + (b*x) + s_2->g;
      } 
    } 
  }
  return v_s;
}

// same as above, but returns the [y, x] location of the other end of the
// back pointer from s, where range of x,y = (-1 to 1). This is used mainly
// for visualization
double* computeCostBP(node* s, node* s_a, node* s_b)
{
  node* s_1;
  node* s_2;
  double c, b, f, v_s, helper;
  double y = 1;
  double x = 1;
  int b_y, b_x;
  
  if(s == NULL || s_a == NULL || s_b == NULL)
  {
    computeCostBPKey[0] = 0;
    computeCostBPKey[1] = 0;
    return computeCostBPKey;  
  }

  if(s->x == s_b->x || s->y == s_b->y) // then s_b is horizontal or vertical neighbor of s, thus, s_a is a diagonal neighbor of s
  {
    s_1 = s_b;  
    s_2 = s_a;
  }
  else
  {
    s_1 = s_a;  
    s_2 = s_b;  
  }
  
  // c is traversal cost of map cell with corners s, s_1, s_2
  c = map[intMin3(s->y, s_1->y, s_2->y)][intMin3(s->x, s_1->x, s_2->x)];

  // b is traversal cost of map cell with corners s and s_1, not s_2
  b_y = intMin2(s->y, s_1->y);
  b_x = intMin2(s->x, s_1->x);
  
  // the above assumes that either c is above b or c is to the left of b, we need to modify things if this is not the case
  if(s_1->y < s_2->y) // then c is actually below b
    b_y--;
  else if(s_1->x < s_2->x)  // then c is actually to the right of b
    b_x--;
  
  // need to take care of edge cases (WIDTH or HEIGHT)
  if(b_y == HEIGHT || b_x == WIDTH || b_y == -1 || b_x == -1)
    b = c;
  else
    b = map[b_y][b_x];

  if(min(c,b) >= LARGE)
  {
    // v_s = LARGE;
    x = 0;
    y = 0;
  }
  else if(s_1->g <= s_2->g)
  {  
    // v_s = min(c,b) + s_1->g;
    y = 0;
  }
  else
  {
    f = s_1->g - s_2->g;
    if(f <= b)
    {
      if(c <= f)
      {
        // v_s = c*sqrt2 + s_2->g;
      }
      else
      {
        y = min(f/sqrt(c*c - f*f),1);
        // v_s = (c*sqrt(1+y*y)) + (f*(1-y)) + s_2->g;
      }
    }
    else
    {
      if(c <= b)
      {
        // v_s = c*sqrt2 + s_2->g;
      }
      else
      {
        x = 1 - min(b/sqrt(c*c - b*b),1);  
        //v_s = (c*sqrt(1+((1-x)*(1-x)))) + (b*x) + s_2->g;
      } 
    } 
  }
 
  // need to figure out which direction x and y are in
  if(s->y == s_1->y) // s_1 is to the left or right of s
  {    
    if(s_1->x > s->x) // s_1 is to the right of s
    {
      // x stays the same
      if(s_2->y > s_1->y) // s_2 is below (greater than) s_1
      {
        // y stays the same
      }
      else  // s_2 is above (less than) s_1
      {
        // y is negative
        y = -y;
      }
        
    }
    else // s_1 is to the left of s
    {
      // x is negative  
      x = -x;
      if(s_2->y > s_1->y) // s_2 is below (greater than) s_1
      {
        // y stays the same
      }
      else  // s_2 is above (less than) s_1
      {
        // y is negative
        y = -y;
      }  
    }
  }
  else // s_1 is up or down of s (x and y are swapped)
  {
    helper = x;
    x = y;
    y = helper;
      
    if(s_1->y > s->y) // s_1 is below (greater than) s
    {
      // y stays the same      
      if(s_2->x > s_1->x) // s_2 is to the right of s_1
      {
        // x stays the same
      }
      else  // s_2 is to the left of s_1
      {
        // x is negative
        x = -x;
      }  
    }
    else // s_1 is above (less than) s
    {
      // y is negative same 
      y = -y;  
      if(s_2->x > s_1->x) // s_2 is to the right of s_1
      {
        // x stays the same
      }
      else  // s_2 is to the left of s_1
      {
        // x is negative
        x = -x;
      }  
    }   
  }
  computeCostBPKey[0] = y;
  computeCostBPKey[1] = x;
  return computeCostBPKey;
}

// function cknbr from the paper, this returns the next neighboring node of
// s in the clockwise direction from s_prime
node* cknbr(node* s, node* s_prime)
{
  int new_x = s_prime->x;
  int new_y = s_prime->y;    
  int delta_x = new_x - s->x;
  int delta_y = new_y - s->y;
  
  if(delta_y == -1)
  {     
    if(delta_x == 1)
      new_y++;    
    else 
      new_x++;      
  }
  else if(delta_y == 1)
  {
    if(delta_x == -1)
      new_y--;
    else 
      new_x--;     
  }
  else if(delta_y == 0)
  {
    if(delta_x == -1)
      new_y--;   
    else 
      new_y++;  
  }
  
  if(new_y > HEIGHT || new_y < 0 || new_x > WIDTH || new_x < 0)
      return NULL;
  
  return &graph[new_y][new_x]; 
}

// function ccknbr from the paper, this returns the next neighboring node of
// s in the counter-clockwise direction from s_prime
node* ccknbr(node* s, node* s_prime)
{
  int new_x = s_prime->x;
  int new_y = s_prime->y;    
  int delta_x = new_x - s->x;
  int delta_y = new_y - s->y;
  
  if(delta_y == 1)
  {     
    if(delta_x == 1)
      new_y--;    
    else 
      new_x++;      
  }
  else if(delta_y == -1)
  {
    if(delta_x == -1)
      new_y++;
    else 
      new_x--;     
  }
  else if(delta_y == 0)
  {
    if(delta_x == 1)
      new_y--;   
    else 
      new_y++;  
  }
  if(new_y > HEIGHT || new_y < 0 || new_x > WIDTH || new_x < 0)
    return NULL;
  
  return &graph[new_y][new_x]; 
}

// this is function UpdateNode(s) from the paper
void updateNode(node* s)
{
  if(s->g != s->rhs)
  {
    if(s->inHeap != true) // I added this check to increase speed
      insert(s,calculateKey(s));
    else
      update(s,calculateKey(s)); 
  }
  else if(s->inHeap == true)
    deleteNodeFromHeap(s);  
}

// this finds the minimum value of computeCost over all neighbors s_prime that are neighbors of s
double findMinRhs(node* s)
{
  node* s_prime;
  int n;
  double minRhs = 2*LARGE;
  double thisC;
  for(n = 0; n < NEIGHBORS; n++)
  {
    s_prime = s->neighbors[n];
    if(s_prime == NULL)
      continue;
    
    thisC = computeCost(s, s_prime, ccknbr(s,s_prime));
    if(thisC < minRhs)
      minRhs = thisC;
  }
  return minRhs;
}

// this finds the node with the minimum value of computeCost over all neighbors s_prime that are neighbors of s
node* findNodeWithMinRhs(node* s)
{
  node* s_prime;
  node* s_min = NULL;
  int n;
  double minRhs = 2*LARGE;
  double thisC;
  
  for(n = 0; n < NEIGHBORS; n++)
  {
    s_prime = s->neighbors[n];
    if(s_prime == NULL)
      continue;
    
    thisC = computeCost(s, s_prime, ccknbr(s,s_prime));
    if(thisC < minRhs)
    {
      minRhs = thisC;
      s_min = s_prime; 
    }
  }
  return s_min;
}


// this is procedure computeShortestPath from the paper (final optimized version), it does the meat of the planning/replanning
void computeShortestPath()
{
  node* s;
  node* s_prime;
  double computedCost;
  int n;
  double rhs_old;
  
  while(keyLess(topKey(),calculateKey(s_start)) || s_start->rhs != s_start->g)
  {
    s = topHeap();
    
    if(s->g >= s->rhs)
    {
      s->g = s->rhs; 
      popHeap();
      
      // for all neighbors
      for(n = 0; n < NEIGHBORS; n++)
      { 
        s_prime = s->neighbors[n];
        if(s_prime == NULL)
          continue;

        if(s_prime->inHeap == false)
        {
          s_prime->g = LARGE;
          s_prime->rhs = LARGE;
        }
        
        rhs_old = s_prime->rhs;
        computedCost = computeCost(s_prime,s,ccknbr(s_prime,s));
        
        if(s_prime->rhs > computedCost)
        {
          s_prime->rhs = computedCost;
          s_prime->bestNeighbor = s;
        }
        
        computedCost = computeCost(s_prime,s,cknbr(s_prime,s));
        
        if(s_prime->rhs > computedCost)
        {
          s_prime->rhs = computedCost;
          s_prime->bestNeighbor = cknbr(s_prime,s);
        }

        if(s->rhs != rhs_old) // is this an error? should it be s_prime->rhs ?
          updateNode(s_prime);
      }
    }
    else
    {   
      s->rhs = findMinRhs(s); // possibility for for speed up here, by combining this and the next
      s->bestNeighbor = findNodeWithMinRhs(s); 
        
      if(s->g < s->rhs) 
      {          
        s->g = LARGE;  
     
        // for all neighbors
        for(n = 0; n < NEIGHBORS; n++)
        {
          s_prime = s->neighbors[n];
          if(s_prime == NULL)
            continue; 
          
          if((s_prime->bestNeighbor == s || s_prime->bestNeighbor == cknbr(s_prime,s)) && s_prime != s_goal) // I added the check for the goal
          {
            if(s_prime->rhs != computeCost(s_prime,s_prime->bestNeighbor,ccknbr(s_prime,s_prime->bestNeighbor)))
            {
              if(s_prime->g < s_prime->rhs || s_prime->inHeap != true)
              {
                s_prime->rhs = LARGE;
                updateNode(s_prime);
              }
              else
              {    
                s_prime->rhs = findMinRhs(s_prime); // possibility for for speed up here, by combining this and the next
                s_prime->bestNeighbor = findNodeWithMinRhs(s_prime);
                updateNode(s_prime);
              }
            }
          }  
        }
      }
      updateNode(s); 
    }
  }
}

// this is function UpdateCellCost(x,c) from the paper, it updates the cost
// of a grid cell at position (y x), then updates all relevant nodes
void updadeCell(int y, int x, double newCost)
{
  node* s;
  node* bestNeighbor;
  node* anotherNeighbor; // actually the other best
  int i;  
  
  // make sure that the update is in the map
  if(y < 0 || y >= HEIGHT || x < 0 || x >= WIDTH)
      return;
  
  if(newCost > map[y][x])
  {
    map[y][x] = newCost; 
      
    // for each node s on a corner of map[y][x]
    for(i = 0; i < corners; i++) 
    {   
      s = &graph[y+deltaY[i]][x+deltaX[i]];
      
      bestNeighbor = s->bestNeighbor;
      if(bestNeighbor == NULL)
        continue;
        
      anotherNeighbor = ccknbr(s, bestNeighbor);
      if(anotherNeighbor == NULL)
        anotherNeighbor = bestNeighbor; // this is a bit of a hack to make the following 'if' statement easier
      
      // if either bptr(s) or ccknbr(s,bptr(s)) is a corner of x
      if(((bestNeighbor->y == y || bestNeighbor->y - 1 == y) && (bestNeighbor->x == x || bestNeighbor->x -1 == x)) ||
         ((anotherNeighbor->y == y || anotherNeighbor->y - 1 == y) && (anotherNeighbor->x == x || anotherNeighbor->x -1 == x)))
      {
        if(s->rhs != computeCost(s, bestNeighbor, anotherNeighbor))
        {
          if(s->g < s->rhs || s->inHeap != true)
          {
            s->rhs = LARGE;
            updateNode(s); 
          }
          else
          {
            s->rhs = findMinRhs(s);
            s->bestNeighbor = findNodeWithMinRhs(s); 
            updateNode(s); 
          }
        } 
      }
    }
  }
  else
  {
    map[y][x] = newCost;  
      
    double rhs_min = LARGE;
    node* s_star = NULL;
    
    // for each node s on a corner of map[y][x]
    for(i = 0; i < corners; i++) 
    {   
      s = &graph[y+deltaY[i]][x+deltaX[i]];
      
      if(s->inHeap == false)
      {
        // don't need this because I am currently initializing all nodes at start
        //  s->g = LARGE;
        //  s->rhs = LARGE;
      }
      else if(s->rhs < rhs_min)
      {
        rhs_min = s->rhs;
        s_star = s;
      }
      
      if(rhs_min != LARGE)
      {
        if(s_star->inHeap != true) // I added this check to increase speed (note that a value of 'closed' is possible)
          insert(s_star,calculateKey(s_star));
        else
          update(s_star,calculateKey(s_star));  
      } 
    }
  } 
}

// one of the two quadratic equation answers
/*
double quadraticPlus(double A, double B, double C)
{
    double sqrt_part = ((B*B)-4*A*C);
    if(sqrt_part < 0)
    {
      if(-sqrt_part < SMALL) // then it should be 0     
        sqrt_part = 0;
      else // it is an imaginary root
      {
        printf("imaginary part +: %f %f %f %f %f \n", A, B, C, ((B*B)-4*A*C), (-B+sqrt((B*B)-4*A*C))/(2*A));
        //getchar();
      }
    } 
   return (-B+sqrt(sqrt_part))/(2*A);  
}
*/

// the other of the two quadratic equation answers
/*
double quadraticMinus(double A, double B, double C)
{
    double sqrt_part = ((B*B)-4*A*C);
    if(sqrt_part < 0)
    {
      if(-sqrt_part < SMALL) // then it should be 0     
        sqrt_part = 0;
      else // it is an imaginary root
      {
        printf("imaginary part +: %f %f %f %f %f \n", A, B, C, ((B*B)-4*A*C), (-B-sqrt((B*B)-4*A*C))/(2*A));
        //getchar();
      }
    } 
   return (-B-sqrt(sqrt_part))/(2*A);  
}
*/

 // This computes the minimum cost from an arbitrary point inside cell c to
// the edge (of c) that connects nodes s_1 and s_2. The node s is also 
// located on a corner of c, and defines a neighboring cell b that can
// also be used en-route to the edge (s_a s_b). Assuming that s is the 
// origin of a coordinate system where the y axis starts at node s and goes 
// along the edge between c and b, and the x axis starts at s and goes
// along the other edge of c, and that side lengths of c are 1, 
// (y_hat, x_hat) is the location of the arbitrary point
// this returns a CellPath containing relevant info
void computePointCostToEdge(double y_hat, double x_hat, node* s, node* s_a, node* s_b, CellPath** subPaths)
{

  node* s_1;
  node* s_2;
  double c, b, x_1, x_2, ybar_1, ybar_2, g_1, g_2, diagonal_g, straight_g, c_prime;
  int b_y, b_x;
  CellPath* subPath = subPaths[0];
  
  if(s == NULL || s_a == NULL || s_b == NULL)
  {    
    subPath->g = LARGE*2;
    subPath->length = 0;
    return;
  }
  
  if(s->x == s_b->x || s->y == s_b->y) // then s_b is horizontal or vertical neighbor of s, thus, s_a is a diagonal neighbor of s
  {
    s_1 = s_b;  
    s_2 = s_a;
  }
  else if(s->x == s_b->x && s->y == s_b->y)
  {
    printf(" This should never happen 0\n");
    //getchar();  
  }
  else
  {
    s_1 = s_a;  
    s_2 = s_b;  
  }
  
  // c is traversal cost of map cell with corners s, s_1, s_2
  c = map[intMin3(s->y, s_1->y, s_2->y)][intMin3(s->x, s_1->x, s_2->x)];

  // b is traversal cost of map cell with corners s and s_1, not s_2
  b_y = intMin2(s->y, s_1->y);
  b_x = intMin2(s->x, s_1->x);
  
  // the above assumes that either c is above b or c is to the left of b, we need to modify things if this is not the case
  if(s_1->y < s_2->y) // then c is actually below b
    b_y--;
  else if(s_1->x < s_2->x)  // then c is actually to the right of b
    b_x--;

  // need to take care of edge cases (WIDTH or HEIGHT)
  if(b_y == HEIGHT || b_x == WIDTH || b_y == -1 || b_x == -1)
    b = c;
  else
    b = map[b_y][b_x];
    
  
  // find min cost of path going only through cell c, need to find the actual of the two possibilities returned by the quadratic equation
  //printf(" here 1 [%f %f] %f %f %f \n", x_hat, y_hat, s_1->g, s_2->g, c);
  
  // remember the original node values, because we will modify them locally if necessary (see below) but want to leave them the same after we are done
  double old_s_1_g = s_1->g;
  double old_s_2_g = s_2->g;
  
  if((s_1->g == LARGE) && (s_2->g == LARGE))
  {
    // printf("this should never happen \n"); // except possible on the first path step of path extraction
    // getchar();
  }
  else if(s_1->g > s_2->g+c) // to avoide getting imaginary answer, need to give this an appropriate g based on s_2
    s_1->g = s_2->g + c;   
  else if(s_2->g > s_1->g+c) // to avoide getting imaginary answer, need to give this an appropriate g based on s_1
    s_2->g = s_1->g + c; 
  
  //printf(" here 2 [%f %f] %f %f %f \n", x_hat, y_hat, s_1->g, s_2->g, c);
      
  //if((s_1->g - s_2->g + c < SMALL) || (s_2->g - s_1->g + c < SMALL)) // then the equation explodes, but we know that the best option is to go straight at the minimum of g1 or g2 
  if(s_1->g - s_2->g == c || s_2->g - s_1->g == c ) // then the equation explodes, but we know that the best option is to go straight at the minimum of g1 or g2 
  {
    if(s_1->g < s_2->g) // go toward s_1
    {
      subPath->x[0] = 0;  
      subPath->local_g = c*sqrt(((x_hat)*(x_hat)) + ((1-y_hat)*(1-y_hat)));
      subPath->g = subPath->local_g + s_1->g;
    }   
    else if(s_2->g < s_1->g) // go toward s_2
    {    
      subPath->x[0] = 1;
      subPath->local_g = c*sqrt(((1-x_hat)*(1-x_hat)) + ((1-y_hat)*(1-y_hat)));
      subPath->g = subPath->local_g + s_2->g;
    }
    else
    {
      printf("This should not happen 1 (is there cost = 0 in the map?)\n");   
      //getchar();
    }
    subPath->y[0] = 1;
    subPath->g_to_edge = subPath->local_g;
    subPath->length = 1;
  }
  else
  {    
    //x_1 = quadraticPlus(1, -2*x_hat, (x_hat*x_hat) - ((1-y_hat)*(1-y_hat))/(((c/(s_1->g - s_2->g))*(c/(s_1->g - s_2->g))) - 1));
    //x_2 = quadraticMinus(1, -2*x_hat, (x_hat*x_hat) - ((1-y_hat)*(1-y_hat))/(((c/(s_1->g - s_2->g))*(c/(s_1->g - s_2->g))) - 1)); 

    c_prime = sqrt(((1-y_hat)*(1-y_hat))/(((c/(s_1->g - s_2->g))*(c/(s_1->g - s_2->g)))-1)); 
    x_1 = x_hat + c_prime;
    x_2 = x_hat - c_prime;  
    
    x_1 = max(min(x_1,1),0);
    x_2 = max(min(x_2,1),0);
    
    g_1 = c*sqrt(((1-y_hat)*(1-y_hat))+((x_1-x_hat)*(x_1-x_hat))) + x_1*s_2->g + (1-x_1)*s_1->g;
    g_2 = c*sqrt(((1-y_hat)*(1-y_hat))+((x_2-x_hat)*(x_2-x_hat))) + x_2*s_2->g + (1-x_2)*s_1->g;
  
    if( g_1 < g_2)
    {
      subPath->x[0] = x_1;
      subPath->local_g = c*sqrt(((1-y_hat)*(1-y_hat))+((x_1-x_hat)*(x_1-x_hat)));
      subPath->g = subPath->local_g + x_1*s_2->g + (1-x_1)*s_1->g;
    }
    else
    {
      subPath->x[0] = x_2;
      subPath->local_g = c*sqrt(((1-y_hat)*(1-y_hat))+((x_2-x_hat)*(x_2-x_hat)));
      subPath->g = subPath->local_g + x_2*s_2->g + (1-x_2)*s_1->g;
    }
    subPath->y[0] = 1;
    subPath->g_to_edge = subPath->local_g;
    subPath->length = 1;
  } 
    
  if(y_hat == 1) // then the point is on the edge that we are trying to get to, nothing will be better than staying at that point
  {
      subPath->length = 0;
      s_1->g = old_s_1_g;
      s_2->g = old_s_2_g;
      return;
  }
  
  if(b < c) // path may involve going through part of cell b (two options)
  {
    subPath = subPaths[1];  
      
    // option 1, the path goes to b and then along b and then across c to s_2. This will only happen if s_2->g is less than s_1->g
    if(s_2->g < s_1->g)
    {
      // need to find the actual of the two possibilities returned by the quadratic equation
      //ybar_1 = quadraticPlus(1,-2,1 - (((1+x_hat)*(1+x_hat))/(((c*c)/(b*b))-1)));
      //ybar_2 = quadraticPlus(1,-2,1 - (((1+x_hat)*(1+x_hat))/(((c*c)/(b*b))-1)));
        
      c_prime = sqrt(((1+x_hat)*(1+x_hat))/(((c*c)/(b*b))-1));  
      ybar_1 = 1 + c_prime;  
      ybar_2 = 1 - c_prime; 
      
      ybar_1 = max(min(ybar_1,1),0);
      ybar_2 = max(min(ybar_2,1),0);
     
      g_1 = b*(ybar_1-y_hat) + c*sqrt(((1-ybar_1)*(1-ybar_1))+((1+x_hat)*(1+x_hat))) + s_2->g;
      g_2 = b*(ybar_2-y_hat) + c*sqrt(((1-ybar_2)*(1-ybar_2))+((1+x_hat)*(1+x_hat))) + s_2->g;
      
      // need to compare to old g value to see if this is any better, if it is, then this path will eventually go to s_2 
      if(min(g_1,g_2) < subPath->g)
      {  
        subPath->x[0] = 0;
        subPath->x[1] = 0;
        
        if(g_1 < g_2)
        {  
          subPath->y[0] = (((1-ybar_1)*x_hat)/(1+x_hat)) + y_hat;
          subPath->y[1] = 1 - (1-ybar_1)/(1+x_hat);         
          diagonal_g = c*sqrt(((1-ybar_1)*(1-ybar_1))+((1+x_hat)*(1+x_hat)));
          straight_g = b*(ybar_1-y_hat);
        } 
        else // g_2 <= g_1
        {
          subPath->y[0] = (((1-ybar_2)*x_hat)/(1+x_hat)) + y_hat;
          subPath->y[1] = 1 - (1-ybar_2)/(1+x_hat);
          diagonal_g = c*sqrt(((1-ybar_2)*(1-ybar_2))+((1+x_hat)*(1+x_hat)));
          straight_g = b*(ybar_2-y_hat);
        }
        
        if(x_hat == 0) // then the first point should be ignored because (subPath->y[0] == y_hat && subPath->x[0] == x_hat), also subPath->g_to_edge == 0
        {
           subPath->y[0] = subPath->y[1]; 
           subPath->x[0] = subPath->x[1]; 
           subPath->x[1] = 1;
           subPath->y[1] = 1;
           subPath->g_to_edge = straight_g;
           subPath->length = 2;
        }   
        else // all points are used
        {
          subPath->x[2] = 1;
          subPath->y[2] = 1;
          subPath->g_to_edge = diagonal_g*x_hat/(1+x_hat); 
          subPath->length = 3;
        }
        subPath->local_g = diagonal_g + straight_g;
        subPath->g = subPath->local_g + s_2->g;
      }
    }
    
    subPath = subPaths[2];
    
    // option 2, the path goes to b and then along b to s_1. This may be better even if s_2->g is less than s_1->g
    // need to find the actual of the two possibilities returned by the quadratic equation
    //ybar_1 = quadraticPlus(1,-2*y_hat, (y_hat*y_hat) - ((x_hat*x_hat)/(((c*c)/(b*b))-1)));
    //ybar_2 = quadraticPlus(1,-2*y_hat, (y_hat*y_hat) - ((x_hat*x_hat)/(((c*c)/(b*b))-1)));
    
    c_prime = sqrt((x_hat*x_hat)/(((c*c)/(b*b))-1));
    ybar_1 = y_hat + c_prime;
    ybar_2 = y_hat - c_prime;
    
    ybar_1 = max(min(ybar_1,1),0);
    ybar_2 = max(min(ybar_2,1),0);
    
    g_1 = b*(1-ybar_1) + c*sqrt(((ybar_1-y_hat)*(ybar_1-y_hat))+(x_hat*x_hat)) + s_1->g;
    g_2 = b*(1-ybar_2) + c*sqrt(((ybar_2-y_hat)*(ybar_2-y_hat))+(x_hat*x_hat)) + s_1->g;
    
    // need to compare to old g value to see if this is any better, if it is, then this path will eventually go to s_1 
    if(min(g_1,g_2) < subPath->g)
    {
      subPath->x[0] = 0;
     
      if(g_1 < g_2)
      {
        subPath->y[0] = ybar_1;  
        subPath->g_to_edge = c*sqrt(((ybar_1-y_hat)*(ybar_1-y_hat))+(x_hat*x_hat));
        subPath->local_g = subPath->g_to_edge + b*(1-ybar_1);  
      }
      else // g_2 <= g_1
      {
        subPath->y[0] = ybar_2;   
          
        subPath->g_to_edge = c*sqrt(((ybar_2-y_hat)*(ybar_2-y_hat))+(x_hat*x_hat));
        subPath->local_g = subPath->g_to_edge + b*(1-ybar_2);        
      }  
      
      if(x_hat == 0) // then the first point should be ignored because (subPath->y[0] == y_hat && subPath->x[0] == x_hat), also subPath->g_to_edge == 0
      {
        subPath->x[0] = 0;
        subPath->y[0] = 1;
        subPath->g_to_edge = subPath->local_g;   
        subPath->length = 1;   
      }   
      else // all points are used
      {
        subPath->x[1] = 0;
        subPath->y[1] = 1;
        subPath->length = 2;
      }
      subPath->g = subPath->local_g + s_1->g;
    }
  }
  
  // reset node values to their original values
  s_1->g = old_s_1_g;
  s_2->g = old_s_2_g;
  
}

// computes the local cost of getting from the point s' to the goal, s' is
// described first in terms of the cell that contains it [cell_y, cell_x]
// and then in terms of y and x both in the range [0 1], describing the 
// local position of the point within the cell.
// Puts cellPaths into a heap so that they are sorted by best cost
void computeLocalPointCost(int cell_y, int cell_x, double y, double x, cpHeap* thisHeap)
{
  node* s;
  node* s_a;
  node* s_b;  
  double x_hat, y_hat; 
  int n, i, j;
  double temp_x, temp_y;
  CellPath* subPaths[3];
  CellPath* subPath;
  
  // there are 4 possible edges to go to (defined by s_a and s_b), and each 
  // edge can potentially involve two other sides (defined by the remaining
  // two nodes), therefore we need to call computePointCostToEdge 8 times
  for(n = 0; n < corners; n++)
  {
    if(cell_y+deltaY[n] < 0 || cell_y+deltaY[n] > HEIGHT || cell_x+deltaX[n] < 0 || cell_x+deltaX[n] > WIDTH)
        continue;    
    s_a = &graph[cell_y+deltaY[n]][cell_x+deltaX[n]];

    if(n == corners - 1)
    {
      if(cell_y+deltaY[0] < 0 || cell_y+deltaY[0] > HEIGHT || cell_x+deltaX[0] < 0 || cell_x+deltaX[0] > WIDTH)
        continue;
      s_b = &graph[cell_y+deltaY[0]][cell_x+deltaX[0]];
    }
    else
    {
      if(cell_y+deltaY[n+1] < 0 || cell_y+deltaY[n+1] > HEIGHT || cell_x+deltaX[n+1] < 0 || cell_x+deltaX[n+1] > WIDTH)
        continue;
      s_b = &graph[cell_y+deltaY[n+1]][cell_x+deltaX[n+1]];
    }  
    // at this point, s_b is clockwise of s_a, relative to the cell
    
    // case 1: s is the next neighbor of s_a that is clockwise of s_b
    s = cknbr(s_a, s_b);

    // the input to computePointCostToEdge is in relative and not absolute terms, 
    // we need to figure out which way x_hat and y_hat go
    if(n == 0)      // s_a is in the upper left
    {
      x_hat = 1 - x;
      y_hat = 1 - y;
    }
    else if(n == 1) // s_a is in the upper right
    {
      x_hat = 1 - y;
      y_hat = x;        
    }
    else if(n == 2) // s_a is in the lower right
    { 
      x_hat = x; 
      y_hat = y;
    }
    else if(n == 3) // s_a is in the lower left
    {
      x_hat = y;  
      y_hat = 1 - x;
    }

    // get CellPaths to put stuff in (there are 3 possibilities)
    for(j = 0; j < 3; j++)
      subPaths[j] = newCellPath(cell_y, cell_x);
    computePointCostToEdge(y_hat, x_hat, s, s_a, s_b, subPaths);

    // the output from computePointCostToEdge is also relative, so we need to figure out which way best_y and best_x go
    for(j = 0; j < 3; j++)
    {
      subPath = subPaths[j]; 
    
      if(n == 0)      // s_a is in the upper left
      {
        for(i = 0; i < subPath->length; i++)
        {
          temp_x = 1 - subPath->x[i];
          temp_y = 1 - subPath->y[i];
          subPath->x[i] = temp_x;
          subPath->y[i] = temp_y;
        }
      }
      else if(n == 1) // s_a is in the upper right
      {
        for(i = 0; i < subPath->length; i++)
        {
          temp_x = subPath->y[i];
          temp_y = 1 - subPath->x[i]; 
          subPath->x[i] = temp_x;
          subPath->y[i] = temp_y;
        }
      }
      else if(n == 2) // s_a is in the lower right
      {
        for(i = 0; i < subPath->length; i++)
        {
          temp_x = subPath->x[i]; 
          temp_y = subPath->y[i];
          subPath->x[i] = temp_x;
          subPath->y[i] = temp_y;
        }
      }
      else if(n == 3) // s_a is in the lower left
      {
        for(i = 0; i < subPath->length; i++)
        {
          temp_x = 1 - subPath->y[i];  
          temp_y = subPath->x[i];
          subPath->x[i] = temp_x;
          subPath->y[i] = temp_y;
        }
      }

      // add this CellPath to the CellPath heap
      if(subPath->length > 0)
        cpAddToHeap(thisHeap, subPath);
      else
        deleteCellPath(subPath);
      
      //cpPrintHeap(thisHeap);
      //cpCheckHeap(thisHeap);
    }
    
    // case 2: s is the next neighbor of s_b that is counter-clockwise of s_a 
    s = ccknbr(s_b, s_a);

    // the input to computePointCostToEdge is in relative and not absolute terms, 
    // we need to figure out which way x_hat and y_hat go
    if(n == 0)      // s_a is in the upper left
    {
      x_hat = x;
      y_hat = 1 - y;
    }
    else if(n == 1) // s_a is in the upper right
    {
      x_hat = y;
      y_hat = x; 
    }
    else if(n == 2) // s_a is in the lower right
    {
      x_hat = 1 - x;
      y_hat = y;   
    }
    else if(n == 3) // s_a is in the lower left
    {
      x_hat = 1 - y;
      y_hat = 1 - x; 
    }

    // get a new CellPath to put stuff in (there are 3 possibilities)
    for(j = 0; j < 3; j++)
      subPaths[j] = newCellPath(cell_y, cell_x);
    computePointCostToEdge(y_hat, x_hat, s, s_a, s_b, subPaths);

    // the output from computePointCostToEdge is also relative, so we need to figure out which way best_y and best_x go
    for(j = 0; j < 3; j++)
    {
      subPath = subPaths[j];   
      
      if(n == 0)      // s_a is in the upper left
      {
        for(i = 0; i < subPath->length; i++)
        {  
          temp_x = subPath->x[i]; 
          temp_y = 1 - subPath->y[i];
          subPath->x[i] = temp_x;
          subPath->y[i] = temp_y;
        }
      }
      else if(n == 1) // s_a is in the upper right
      {
        for(i = 0; i < subPath->length; i++)
        {
          temp_x = subPath->y[i];
          temp_y = subPath->x[i];
          subPath->x[i] = temp_x;
          subPath->y[i] = temp_y;
        }
      }
      else if(n == 2) // s_a is in the lower right
      {
        for(i = 0; i < subPath->length; i++)
        {  
          temp_x = 1 - subPath->x[i];
          temp_y = subPath->y[i]; 
          subPath->x[i] = temp_x;
          subPath->y[i] = temp_y;
        }
      }     
      else if(n == 3) // s_a is in the lower left
      {
        for(i = 0; i < subPath->length; i++)
        {  
          temp_x = 1 - subPath->y[i];
          temp_y = 1 - subPath->x[i]; 
          subPath->x[i] = temp_x;
          subPath->y[i] = temp_y;
        }
      }

      // add this CellPath to the CellPath heap
      if(subPath->length > 0)
        cpAddToHeap(thisHeap,subPath);
      else
        deleteCellPath(subPath);
      
      //cpPrintHeap(thisHeap);
      //cpCheckHeap(thisHeap);
      //getchar();
    }
  }
}

// runs computeLocalPointCost for each possible cell, puts local paths int
// the cellPath heap 
void computeAllLocalPointCosts(double pos_y, double pos_x, cpHeap* thisHeap)
{
  int int_pos_x = (int)pos_x;
  int int_pos_y = (int)pos_y; 
  int lr, ud;
  int path_cell_x, path_cell_y;
  
  double path_bot_x, path_bot_y;

  if(pos_x == (double)int_pos_x) // then on a vertical edge, need to check the cells to the left and right of the robot
    lr = 1;
  else //only need to check the cells with x == int_pos_x
    lr = 0;
    
  while( lr!= -1)
  {
    // figure out the cell and robot x coordinates for this particular cell 
    if(lr == 1) // check the cells with x == int_pos_x - 1
    {
      path_cell_x = int_pos_x - 1;
      path_bot_x = 1;
    }
    else // lr == 0, check the cells with x == int_pos_x
    {
      path_cell_x = int_pos_x;
      path_bot_x = pos_x - (double)int_pos_x;
    }
          
    if( path_cell_x < 0 || path_cell_x > WIDTH) // then outside of map
    {
      lr--;
      continue;
    }
      
    if(pos_y == (double)int_pos_y) // then on a horizontal edge, need to check the cells to the top and bottom of the robot
      ud = 1;
    else //only need to check the cells with y == int_pos_y
      ud = 0;

    while(ud != -1)
    {
      // figure out the cell and robot y coordinates for this particular cell 
      if(ud == 1) // check the cells with y == int_pos_y - 1
      {
        path_cell_y = int_pos_y - 1;
        path_bot_y = 1;
      }
      else // ud == 0, check the cells with y == int_pos_y
      {
        path_cell_y = int_pos_y;
        path_bot_y = pos_y - (double)int_pos_y;
      }
 
      if( path_cell_y < 0 || path_cell_y > HEIGHT) // then outside of map
      {
        ud--;
        continue;
      } 
  
      // the folowing function puts all possible transitions out of this cell into the CellPath heap
      computeLocalPointCost(path_cell_y, path_cell_x, path_bot_y, path_bot_x, thisHeap);

      ud--;  
    }
    lr--;
  }   
}

#ifndef NOGLUT
// draws the graph back links on the maze at height z_height
void drawGraphBackLinks(double* color, double z_height)
{
  node* anotherBest;
  double* endPoints;
  glPushMatrix();
  
  glTranslatef(-1+1/((double)WIDTH),1-1/((double)HEIGHT),z_height);
  glScaled(2/((double)WIDTH),-2/((double)HEIGHT),1); 
  glTranslatef(-.5,-.5,0);  
  
  glColor3f(color[0],color[1],color[2]);
  glBegin(GL_LINES);
  
  int r, c;
  for (r = 0; r < HEIGHT+1; r++)
  {    
	for (c = 0; c < WIDTH+1; c++)
	{
      if(graph[r][c].bestNeighbor != NULL)
      { 
        anotherBest = ccknbr(&graph[r][c],graph[r][c].bestNeighbor);
        if(anotherBest == NULL)
          continue;
        
        endPoints = computeCostBP(&graph[r][c], graph[r][c].bestNeighbor, anotherBest);
        
        glVertex3d((double)graph[r][c].x+endPoints[1],(double)graph[r][c].y+endPoints[0],0);
        glVertex3d((double)graph[r][c].x,(double)graph[r][c].y,0);
      }
    }
  }
  glEnd();
  glPopMatrix();    
}

// draws the back link of the node at height z_height
void drawNodeBackLinks(node* s, double* color, double z_height)
{
  node* anotherBest;
  double* endPoints;
  glPushMatrix();
  
  glTranslatef(-1+1/((double)WIDTH),1-1/((double)HEIGHT),z_height);
  glScaled(2/((double)WIDTH),-2/((double)HEIGHT),1); 
  glTranslatef(-.5,-.5,0);  
  
  glColor3f(color[0],color[1],color[2]);
  glBegin(GL_LINES);
  
 
  if(s->bestNeighbor != NULL)
  { 
    anotherBest = ccknbr(s,s->bestNeighbor);
    if(anotherBest != NULL)
    {
      endPoints = computeCostBP(s, s->bestNeighbor, anotherBest);

      glVertex3d((double)s->x+endPoints[1],(double)s->y+endPoints[0],0);
      glVertex3d((double)s->x,(double)s->y,0);
    } 
  }
  glEnd();
  glPopMatrix();
}
#endif

// this extracts a path from the graph (no look ahead)
MapPath* extractPath()
{  
  double pos_x = robot_pos_x;
  double pos_y = robot_pos_y;
  int int_pos_x = (int)pos_x;
  int int_pos_y = (int)pos_y; 
  double totalPathCost = 0;
  CellPath* topCellPath;
  MapPath* robotsPath = newMapPath(0); 
  robotsPath = addPathEnd(robotsPath, pos_y, pos_x); // this remembers where the robot has moved, used for graphics
  
  
  //printf("%f %f %d %d \n", pos_x, pos_y, int_pos_x, int_pos_y);
  
  // go toward the goal until there
  #ifdef R2GOAL         
  printf(" warning: extractPath() is not safe to use with a real (i.e. not integer) goal \n");
  while(abs(pos_x - goalRM) >= 1 || abs(pos_y - goalRN) >= 1)
  #else
  while(pos_x != (double)s_goal->x || pos_y != (double)s_goal->y)
  #endif
  {   
      
 //   printf("%f %f %f %f \n", pos_x, pos_y, (double)s_goal->x, (double)s_goal->y);  
      
    computeAllLocalPointCosts(pos_y, pos_x, primaryCellPathHeap);

    // extract the best path found
    topCellPath = cpPopHeap(primaryCellPathHeap);
    
    if(topCellPath == NULL)
       printf("problem, no best neighbor found \n");
    
    if(((double)topCellPath->cell_x+topCellPath->x[0] == pos_x) && ((double)topCellPath->cell_y+topCellPath->y[0] == pos_y))
    {
      printf("problem? next point is this point\n");   
      getchar();
    }
    
    pos_x = (double)topCellPath->cell_x+topCellPath->x[0];
    pos_y = (double)topCellPath->cell_y+topCellPath->y[0];      
    robotsPath = addPathEnd(robotsPath, pos_y, pos_x); // this remembers where the robot has moved, used for graphics

    totalPathCost = totalPathCost + topCellPath->g_to_edge;
    int_pos_x = (int)pos_x;
    int_pos_y = (int)pos_y; 
    
    // because of numerical issues:  
    if((pos_x - (double)int_pos_x < SMALL) && (pos_y - (double)int_pos_y < SMALL))
    {
      //printf("numerical 1 \n");
      pos_x = (double)int_pos_x;
      pos_y = (double)int_pos_y;
    }
    else if(((double)(int_pos_x+1) - pos_x < SMALL) && (pos_y - (double)int_pos_y < SMALL))
    {
       //printf("numerical 2 \n");  
      int_pos_x = int_pos_x+1; 
      pos_x = (double)int_pos_x;
      pos_y = (double)int_pos_y;  
    }
    else if(((double)(int_pos_x+1) - pos_x < SMALL) && (((double)int_pos_y+1) - pos_y < SMALL))
    {
       //printf("numerical 3 \n");  
      int_pos_x = int_pos_x+1; 
      int_pos_y = int_pos_y+1;
      pos_x = (double)int_pos_x;
      pos_y = (double)int_pos_y;  
    }    
    else if((pos_x - (double)int_pos_x < SMALL) && (((double)int_pos_y+1) - pos_y < SMALL))
    {
      //printf("numerical 4 \n");  
      int_pos_y = int_pos_y+1;
      pos_x = (double)int_pos_x;
      pos_y = (double)int_pos_y;  
    }  
    
    /*---------- start of graphics stuff ---------*/
    //if (DISPLAYGRAPHICS == 1)
    #ifndef  NOGLUT
    //{
    //clearDisplay();
    ////printMap(); // on command line
    ////printHeap();
    ////checkHeap();
    //drawMap(-1,-.5);
    ////drawField(-1,-.5);
    //drawHeapOnMaze(.5);
    //drawGraphBackLinks(clrB, .6);
    //drawPathOnMaze(robotsPath, clrW, .75); // draw the path
    ////printPath(robotsPath); // on command line
    //flushDisplay();
    //glutMainLoopEvent();
    //}
    //getchar();
    #endif
    /*----------- end of graphics stuff ----------*/ 
    
    // re-initialize the cellPath look-ahead heap
    deleteCellPath(topCellPath);
    cpCleanHeap(primaryCellPathHeap);  
  }  
  
  #ifdef R2GOAL
    robotsPath = addPathEnd(robotsPath, goalRN, goalRM); // add goal to the end of the path
    // add final segment cost
    double final_c = map[intMin2((int)goalRN, pos_y)][intMin2((int)goalRM, pos_x)];
    if(goalRN == pos_y && goalRM != pos_x && pos_y > 0)
    {
      if(map[intMin2((int)goalRN, pos_y)-1][intMin2((int)goalRM, pos_x)] < final_c)
         final_c = map[intMin2((int)goalRN, pos_y)-1][intMin2((int)goalRM, pos_x)];
    }
    else if(goalRM == pos_x && goalRN != pos_y && pos_x > 0)
    {
      if(map[intMin2((int)goalRN, pos_y)][intMin2((int)goalRM, pos_x)-1] < final_c)
         final_c = map[intMin2((int)goalRN, pos_y)][intMin2((int)goalRM, pos_x)-1];  
    }
    
    double final_d = sqrt((goalRN - pos_y)*(goalRN - pos_y) + (goalRM - pos_x)*(goalRM - pos_x));
    totalPathCost = totalPathCost + final_c*final_d;
  #endif
  
  robotsPath = removeRepeatedPoints(robotsPath);
  //printf("No look ahead total path cost = %f \n",totalPathCost);
  return robotsPath;
}

// this extracts a path from the graph (with one look ahead)
MapPath* extractPathOneLookahead()
{  
  double pos_x = robot_pos_x;
  double pos_y = robot_pos_y;
  int int_pos_x = (int)pos_x;
  int int_pos_y = (int)pos_y; 
  double totalPathCost = 0;
  double lookAhead_pos_y;
  double lookAhead_pos_x;
  double x_min, x_max, y_min, y_max;
  
  CellPath* topCellPath;
  CellPath* bestCellPath;
  CellPath* lookAheadCellPath;
  
  MapPath* robotsPath = newMapPath(0); 
  robotsPath = addPathEnd(robotsPath, pos_y, pos_x); // this remembers where the robot has moved, used for graphics
    
  // go toward the goal until there
  #ifdef R2GOAL
  while(abs(pos_x - goalRM) >= 1 || abs(pos_y - goalRN) >= 1)
  printf(" warning: extractPathOneLookahead() is not safe to use with a real (i.e. not integer) goal \n");
  #else
  while(pos_x != (double)s_goal->x || pos_y != (double)s_goal->y)
  #endif
  {       
    //printf("%f %f %f %f \n", pos_x, pos_y, (double)s_goal->x, (double)s_goal->y);  
   
    computeAllLocalPointCosts(pos_y, pos_x, primaryCellPathHeap);
    
    // loop until we break
    while(1 == 1)
    {
      // extract the best path found
      topCellPath = cpPopHeap(primaryCellPathHeap);  
        
      if(topCellPath == NULL)
        printf("problem, no best neighbor found \n");
    
      if(((double)topCellPath->cell_x+topCellPath->x[0] == pos_x) && ((double)topCellPath->cell_y+topCellPath->y[0] == pos_y))
      {
        printf("problem? next point is this point\n");   
        getchar();
      }
    
      // now we want to do a 1 step look ahead to make sure that this point is still the best
      bestCellPath = topCellPath;                   // this is the current best
      topCellPath = cpTopHeap(primaryCellPathHeap); // this is the current second best
      
      lookAhead_pos_y = (double)bestCellPath->cell_y + bestCellPath->y[0];
      lookAhead_pos_x = (double)bestCellPath->cell_x + bestCellPath->x[0];
      
      if(lookAhead_pos_y == pos_y && lookAhead_pos_x == pos_x)
      {
        printf("This should not happen 2\n");
        getchar();
      }
            
      // check if the look ahead point is the goal, if so, then we know that this was the optimal path
      #ifdef R2GOAL
      if(((double)bestCellPath->cell_y + bestCellPath->y[0] == goalRN) && ((double)bestCellPath->cell_x + bestCellPath->x[0] == goalRM))
        break;        
      #else
      if(((double)bestCellPath->cell_y + bestCellPath->y[0] == (double)s_goal->y) && ((double)bestCellPath->cell_x + bestCellPath->x[0] == (double)s_goal->x))
        break;
      #endif
      
      computeAllLocalPointCosts(lookAhead_pos_y, lookAhead_pos_x, secondaryCellPathHeap);
      lookAheadCellPath = cpTopHeap(secondaryCellPathHeap); // this is the path from the look-ahead point
      
      // make sure that a look ahead point on a corner doesn't lead to a path where the next point is on the edge of the cell containing the robot (corners are ok)
      if(lookAhead_pos_y == (double)((int)lookAhead_pos_y) && lookAhead_pos_x == (double)((int)lookAhead_pos_x)) // look ahead point is on a corner
      {
          
        if(lookAheadCellPath->x[0] == (double)((int)(lookAheadCellPath->x[0])) && lookAheadCellPath->y[0] == (double)((int)(lookAheadCellPath->y[0]))) // next point is also on a corner
        {
            // corners are ok
        }
        else
        { 
          if((double)int_pos_y == pos_y) // then the robot is on a horizontal edge
            y_min = pos_y - 1;
          else
            y_min = (double)int_pos_y;
          y_max = (double)(int_pos_y+1);   
              
          if((double)int_pos_x == pos_x) // then the robot is on a vertical edge
            x_min = pos_x - 1;
          else
            x_min = (double)int_pos_x;
          x_max = (double)(int_pos_x+1);     
             
          // if the first point after the look ahead point is in this range, then we don't want to use the look ahead point (ever)
          if(x_min <= lookAheadCellPath->x[0] && lookAheadCellPath->x[0] <= x_max && y_min <= lookAheadCellPath->y[0] && lookAheadCellPath->y[0] <= y_max)
          {
              printf(" what do you know, we are here \n");
              getchar();
            deleteCellPath(bestCellPath);
            cpCleanHeap(secondaryCellPathHeap);
            continue;
          }      
        }      
      }
      
      if(bestCellPath->g_to_edge + lookAheadCellPath->g > topCellPath->g + SMALL) // note SMALL is added because I encountered some numerical issues here
      { 
        // then the look ahead fails, so we'll put what we thought might be the best back into the heap after modifying its values  
        bestCellPath->g = bestCellPath->g_to_edge + lookAheadCellPath->g; 
        cpAddToHeap(primaryCellPathHeap, bestCellPath);  
      }
      else // it is still the best
      {
        cpCleanHeap(secondaryCellPathHeap);
        break;
      }
      cpCleanHeap(secondaryCellPathHeap); 
    }
    
    pos_x = (double)bestCellPath->cell_x+bestCellPath->x[0];
    pos_y = (double)bestCellPath->cell_y+bestCellPath->y[0];      
    robotsPath = addPathEnd(robotsPath, pos_y, pos_x); // this remembers where the robot has moved, used for graphics

    totalPathCost = totalPathCost + bestCellPath->g_to_edge;
    int_pos_x = (int)pos_x;
    int_pos_y = (int)pos_y; 
    
    // because of numerical issues:  
    if((pos_x - (double)int_pos_x < SMALL) && (pos_y - (double)int_pos_y < SMALL))
    {
      //printf("numerical 1 \n");
      pos_x = (double)int_pos_x;
      pos_y = (double)int_pos_y;
    }
    else if(((double)(int_pos_x+1) - pos_x < SMALL) && (pos_y - (double)int_pos_y < SMALL))
    {
       //printf("numerical 2 \n");  
      int_pos_x = int_pos_x+1; 
      pos_x = (double)int_pos_x;
      pos_y = (double)int_pos_y;  
    }
    else if(((double)(int_pos_x+1) - pos_x < SMALL) && (((double)int_pos_y+1) - pos_y < SMALL))
    {
       //printf("numerical 3 \n");  
      int_pos_x = int_pos_x+1; 
      int_pos_y = int_pos_y+1;
      pos_x = (double)int_pos_x;
      pos_y = (double)int_pos_y;  
    }    
    else if((pos_x - (double)int_pos_x < SMALL) && (((double)int_pos_y+1) - pos_y < SMALL))
    {
      //printf("numerical 4 \n");  
      int_pos_y = int_pos_y+1;
      pos_x = (double)int_pos_x;
      pos_y = (double)int_pos_y;  
    }      
        
    /*---------- start of graphics stuff ---------*/
    #ifndef  NOGLUT
    //if (DISPLAYGRAPHICS == 1)
    //{
    //clearDisplay();
    ////printMap(); // on command line
    ////printHeap();
    ////checkHeap();
    //drawMap(-1,-.5);
    ////drawField(-1,-.5);
    //drawHeapOnMaze(.5);
    //drawGraphBackLinks(clrB, .6);
    //drawPathOnMaze(robotsPath, clrW, .75); // draw the path
    //printPath(robotsPath); // on command line
    //flushDisplay();
    //glutMainLoopEvent();
    //}
    //getchar();
    #endif
    /*----------- end of graphics stuff ----------*/ 
    
    // re-initialize the cellPath look-ahead heap
    cpCleanHeap(primaryCellPathHeap);  
    deleteCellPath(bestCellPath);
  }  
  
  #ifdef R2GOAL
    robotsPath = addPathEnd(robotsPath, goalRN, goalRM); // add goal to the end of the path
    // add final segment cost
    double final_c = map[intMin2((int)goalRN, pos_y)][intMin2((int)goalRM, pos_x)];
    if(goalRN == pos_y && goalRM != pos_x && pos_y > 0)
    {
      if(map[intMin2((int)goalRN, pos_y)-1][intMin2((int)goalRM, pos_x)] < final_c)
         final_c = map[intMin2((int)goalRN, pos_y)-1][intMin2((int)goalRM, pos_x)];
    }
    else if(goalRM == pos_x && goalRN != pos_y && pos_x > 0)
    {
      if(map[intMin2((int)goalRN, pos_y)][intMin2((int)goalRM, pos_x)-1] < final_c)
         final_c = map[intMin2((int)goalRN, pos_y)][intMin2((int)goalRM, pos_x)-1];  
    }
    
    double final_d = sqrt((goalRN - pos_y)*(goalRN - pos_y) + (goalRM - pos_x)*(goalRM - pos_x));
    totalPathCost = totalPathCost + final_c*final_d;
  #endif
  
  robotsPath = removeRepeatedPoints(robotsPath);
  //printf("One look ahead total path cost = %f \n",totalPathCost);
  return robotsPath;
} 
  

// this extracts a path from the graph using the method I developed
MapPath* extractPathMine(int bad_range_flag)
{  
  double pos_x = robot_pos_x;
  double pos_y = robot_pos_y;
  int int_pos_x = (int)pos_x;
  int int_pos_y = (int)pos_y; 
  double totalPathCost = 0;
  double lookAhead_pos_y;
  double lookAhead_pos_x;
  double x_min, x_max, y_min, y_max;
  double at_x, at_y;
  int int_at_x, int_at_y;
  node* anotherBest;
  double* endPoints;
  int using_bad_range = bad_range_flag; // 1 = don't use gradient if in the bad angle range
  double angle_min = 20;// 22; //22 ; //atan(1/2)-.5;//24;
  double angle_max = 30;//27; //28 ; //atan(1/2)+.5;//27;
  int using_min_only = 0; // 1 = only use gradient if moving between two min cost nodes 


  CellPath* topCellPath;
  CellPath* bestCellPath;
  CellPath* lookAheadCellPath;
  
  MapPath* robotsPath = newMapPath(0); 
  robotsPath = addPathEnd(robotsPath, pos_y, pos_x); // this remembers where the robot has moved, used for graphics
    
  // go toward the goal until there
  #ifdef R2GOAL
  while(abs(pos_x - goalRM) >= 1 || abs(pos_y - goalRN) >= 1)
  #else
  while(pos_x != (double)s_goal->x || pos_y != (double)s_goal->y)
  #endif
  { 
    at_x = pos_x;
    at_y = pos_y;
    int_at_x = (int)at_x; 
    int_at_y = (int)at_y;

    computeAllLocalPointCosts(pos_y, pos_x, primaryCellPathHeap);
      
    // loop until we break
    while(1 == 1)
    {
      // extract the best path found
      topCellPath = cpPopHeap(primaryCellPathHeap);  
        
      if(topCellPath == NULL)
        printf("problem, no best neighbor found \n");
    
      if(((double)topCellPath->cell_x+topCellPath->x[0] == pos_x) && ((double)topCellPath->cell_y+topCellPath->y[0] == pos_y))
      {
        printf("problem? next point is this point\n");   
        getchar();
      }
    
      // now we want to do a 1 step look ahead to make sure that this point is still the best
      bestCellPath = topCellPath;                   // this is the current best
      topCellPath = cpTopHeap(primaryCellPathHeap); // this is the current second best
      
      lookAhead_pos_y = (double)bestCellPath->cell_y + bestCellPath->y[0];
      lookAhead_pos_x = (double)bestCellPath->cell_x + bestCellPath->x[0];
      
      if(lookAhead_pos_y == pos_y && lookAhead_pos_x == pos_x)
      {
        printf("This should not happen 3 \n");
        getchar();
      }
            
      // check if the look ahead point is the goal, if so, then we know that this was the optimal path
      #ifdef R2GOAL
      if(((double)bestCellPath->cell_y + bestCellPath->y[0] == goalRN) && ((double)bestCellPath->cell_x + bestCellPath->x[0] == goalRM))
        break;        
      #else
      if(((double)bestCellPath->cell_y + bestCellPath->y[0] == (double)s_goal->y) && ((double)bestCellPath->cell_x + bestCellPath->x[0] == (double)s_goal->x))
        break;
      #endif 
      
      computeAllLocalPointCosts(lookAhead_pos_y, lookAhead_pos_x, secondaryCellPathHeap);
      lookAheadCellPath = cpTopHeap(secondaryCellPathHeap); // this is the path from the look-ahead point
      
      // make sure that a look ahead point on a corner doesn't lead to a path where the next point is on the edge of the cell containing the robot (corners are ok)
      /*if(lookAhead_pos_y == (double)((int)lookAhead_pos_y) && lookAhead_pos_x == (double)((int)lookAhead_pos_x)) // look ahead point is on a corner
      {
          
        if(lookAheadCellPath->x[0] == (double)((int)(lookAheadCellPath->x[0])) && lookAheadCellPath->y[0] == (double)((int)(lookAheadCellPath->y[0]))) // next point is also on a corner
        {
            // corners are ok
        }
        else
        { 
          if((double)int_pos_y == pos_y) // then the robot is on a horizontal edge
            y_min = pos_y - 1;
          else
            y_min = (double)int_pos_y;
          y_max = (double)(int_pos_y+1);   
              
          if((double)int_pos_x == pos_x) // then the robot is on a vertical edge
            x_min = pos_x - 1;
          else
            x_min = (double)int_pos_x;
          x_max = (double)(int_pos_x+1);     
             
          // if the first point after the look ahead point is in this range, then we don't want to use the look ahead point (ever)
          if(x_min <= lookAheadCellPath->x[0] && lookAheadCellPath->x[0] <= x_max && y_min <= lookAheadCellPath->y[0] && lookAheadCellPath->y[0] <= y_max)
          {
              printf(" what do you know, we are here my %f %f/\n",y_min,x_min);  // reinsert into heap with new point and cost based on look ahead
              getchar();
            deleteCellPath(bestCellPath);
            cpCleanHeap(secondaryCellPathHeap);
            continue;
          }      
        }      
      }*/
      
      if(bestCellPath->g_to_edge + lookAheadCellPath->g > topCellPath->g + SMALL) // note SMALL is added because I encountered some numerical issues here
      { 
        // then the look ahead fails, so we'll put what we thought might be the best back into the heap after modifying its values  
        bestCellPath->g = bestCellPath->g_to_edge + lookAheadCellPath->g; 
        cpAddToHeap(primaryCellPathHeap, bestCellPath);  
      }
      else // it is still the best
      {
        cpCleanHeap(secondaryCellPathHeap);
        break;
      }
      cpCleanHeap(secondaryCellPathHeap); 
    }

    pos_x = (double)bestCellPath->cell_x+bestCellPath->x[0];
    pos_y = (double)bestCellPath->cell_y+bestCellPath->y[0];

    totalPathCost = totalPathCost + bestCellPath->g_to_edge; 
    int_pos_x = (int)pos_x;
    int_pos_y = (int)pos_y; 

    // because of numerical issues:  
    if((pos_x - (double)int_pos_x < SMALL) && (pos_y - (double)int_pos_y < SMALL))
    {
      //printf("numerical 1 \n");
      pos_x = (double)int_pos_x;
      pos_y = (double)int_pos_y;
    }
    else if(((double)(int_pos_x+1) - pos_x < SMALL) && (pos_y - (double)int_pos_y < SMALL))
    {
       //printf("numerical 2 \n");  
      int_pos_x = int_pos_x+1; 
      pos_x = (double)int_pos_x;
      pos_y = (double)int_pos_y;  
    }
    else if(((double)(int_pos_x+1) - pos_x < SMALL) && (((double)int_pos_y+1) - pos_y < SMALL))
    {
       //printf("numerical 3 \n");  
      int_pos_x = int_pos_x+1; 
      int_pos_y = int_pos_y+1;
      pos_x = (double)int_pos_x;
      pos_y = (double)int_pos_y;  
    } 
    else if((pos_x - (double)int_pos_x < SMALL) && (((double)int_pos_y+1) - pos_y < SMALL))
    {
      //printf("numerical 4 \n");  
      int_pos_y = int_pos_y+1;
      pos_x = (double)int_pos_x;
      pos_y = (double)int_pos_y;  
    }


    #ifdef R2GOAL
    if(goalRN == pos_y && goalRM == pos_x)         
    #else
    if(s_goal == &graph[int_pos_y][int_pos_x])
    #endif
    {
       //printf(" here \n");
       //getchar();
    }
    // if on an edge and back pointers from nodes on either end of this edge converge 
    // and end on a parallel edge to this edge, then use gradient method
    else if(at_x == (double)int_at_x && at_y != (double)int_at_y)
    {

      // on a vertical line but not a horizontal line
      double low_x = -1;
      double low_y = -1;
      double low_path_type = -1;
      double high_x = -1;
      double high_y = -1;
      double high_path_type = -1;

      // find back pointer from low end
      if(graph[int_at_y][int_at_x].bestNeighbor != NULL)
      {
        anotherBest = ccknbr(&graph[int_at_y][int_at_x],graph[int_at_y][int_at_x].bestNeighbor);
        if(anotherBest != NULL)
        {
          endPoints = computeCostBP(&graph[int_at_y][int_at_x], graph[int_at_y][int_at_x].bestNeighbor, anotherBest);

          low_y = (double)int_at_y+endPoints[0];
          low_x = (double)int_at_x+endPoints[1];
          low_path_type = endPoints[2];
        }
      }

      // find back pointer from high end
      if(int_at_y+1 <= HEIGHT) 
      {
        if(graph[int_at_y+1][int_at_x].bestNeighbor != NULL)
        {
          anotherBest = ccknbr(&graph[int_at_y+1][int_at_x],graph[int_at_y+1][int_at_x].bestNeighbor);
          if(anotherBest != NULL)
          {
            endPoints = computeCostBP(&graph[int_at_y+1][int_at_x], graph[int_at_y+1][int_at_x].bestNeighbor, anotherBest);

             high_y = (double)int_at_y+1+endPoints[0];
             high_x = (double)int_at_x+endPoints[1];
             high_path_type = endPoints[2];
          }
        }
      }

      if(low_x == high_x && high_y - low_y < 1 && low_path_type != 2 && high_path_type !=2)
      {
        // both end's back pointers go to the same edge, they are converging, and only have one segment each

        // finding grid on other side of edge
        double temp_x;
        if(pos_x > at_x)
          temp_x = pos_x;
        else // pos_x < at_x
          temp_x = pos_x - 1;

        if(temp_x >= WIDTH)
          temp_x = WIDTH - 1;
        if(temp_x < 0)
          temp_x = 0;

        if(using_min_only == 1 && (map[(int)min(pos_y,at_y)][(int)min(pos_x,at_x)] != 1 || map[(int)min(pos_y,at_y)][(int)temp_x] !=1))
        { 

        }
        else if(pos_x == (double)int_pos_x && pos_y == (double)int_pos_y && map[(int)min(pos_y,at_y)][(int)min(pos_x,at_x)] < map[(int)min(pos_y,at_y)][(int)temp_x])
        {
          // because we lack look ahead, there could be a problem if the grid on the other side of 
          // the edge (to which the back pointers end at) has a higher cost value than the one on 
          // the current grid. This is only a concern if (due to look ahead) pose is current at a corner
          // so if that happens, we don't use the gradient stuff.
        }
        else
        {
          // find interpolation between high_y and low_y
          double new_pos_y = low_y + ((at_y - (double)int_at_y)*(high_y - low_y));
          double new_pos_x = low_x;

          // the gradient method is also worse than the linear interpolation method if the resulting angle between the 
          // (gradient) back pointer and the closest axis is in between 23 and 28 degrees.
          //double delta_y = max(at_y - pos_y, pos_y - at_y);
          //double delta_x = max(at_x - pos_x, pos_x - at_x);
          double delta_y = max(at_y - new_pos_y, new_pos_y - at_y);
          double delta_x = max(at_x - new_pos_x, new_pos_x - at_x);
          double angle;
          if(delta_y == 0 || delta_x == 0)
            angle = 0;
          else if(delta_y > delta_x)
            angle = atan(delta_x/delta_y)/(PI)*180;
          else
            angle = atan(delta_y/delta_x)/(PI)*180;

          if(angle_min < angle && angle < angle_max && using_bad_range == 1)
          {
            // in the bad range
          }
          else
          {
            // reset pos_y based on interpolation between high_y and low_y
            pos_y = new_pos_y;
            pos_x = new_pos_x;

            // if this is in a grid that is up or down of where we started, we need to solve for the exit point of this grid
            if((int)pos_y != int_at_y) // && high_y != low_y)
            { 
              int crossed_y;
              //printf("changed vert grid \n");

              // find the y value of the horizontal line that we crossed
              if(at_y > pos_y) // then we crossed int_at_y
                crossed_y = (double)int_at_y;
              else // pos_y > at_y and we crossed (int)pos_y = int_at_y + 1
                crossed_y = (double)int_at_y + 1;

              if(pos_x > at_x)
                pos_x = at_x + (crossed_y - at_y)/(pos_y - at_y);
              else
                pos_x = at_x - (crossed_y - at_y)/(pos_y - at_y);

              pos_y = crossed_y;
            }

            // readjust totalPathCost
            totalPathCost = totalPathCost - bestCellPath->g_to_edge + map[(int)min(pos_y, at_y)][(int)min(pos_x,at_x)]*sqrt((pos_x - at_x)*(pos_x - at_x) + (pos_y - at_y)*(pos_y - at_y)); 

            //printf("on vert edge, gradient \n");
          }
        }
      }
    }
    else if(at_x != (double)int_at_x && at_y == (double)int_at_y)
    {
      // on a horizontal line but not a vertical line
      double low_x = -1;
      double low_y = -1;
      double low_path_type = -1;
      double high_x = -1;
      double high_y = -1;
      double high_path_type = -1;

      // find back pointer from low end
      if(graph[int_at_y][int_at_x].bestNeighbor != NULL)
      { 
        anotherBest = ccknbr(&graph[int_at_y][int_at_x],graph[int_at_y][int_at_x].bestNeighbor);
        if(anotherBest != NULL)
        {
          endPoints = computeCostBP(&graph[int_at_y][int_at_x], graph[int_at_y][int_at_x].bestNeighbor, anotherBest);

          low_y = (double)int_at_y+endPoints[0];
          low_x = (double)int_at_x+endPoints[1];
          low_path_type = endPoints[2];
        } 
      }

      // find back pointer from high end
      if(int_at_x <= WIDTH)
      {
        if(graph[int_at_y][int_at_x+1].bestNeighbor != NULL)
        { 
          anotherBest = ccknbr(&graph[int_at_y][int_at_x+1],graph[int_at_y][int_at_x+1].bestNeighbor);
          if(anotherBest != NULL)
          {
            endPoints = computeCostBP(&graph[int_at_y][int_at_x+1], graph[int_at_y][int_at_x+1].bestNeighbor, anotherBest);

            high_y = (double)int_at_y+endPoints[0];
            high_x = (double)int_at_x+1+endPoints[1];
            high_path_type = endPoints[2];
          } 
        }
      }

      if(low_y == high_y && high_x - low_x < 1 && low_path_type != 2 && high_path_type !=2)
      {
        // both end's back pointers go to the same edge, they are converging, and only have one segment each

        // finding grid on other side of edge
        double temp_y;
        if(pos_y > at_y)
          temp_y = pos_y;
        else // pos_y < at_y
          temp_y = pos_y - 1;

        if(temp_y >= HEIGHT)
          temp_y = HEIGHT - 1;
        if(temp_y < 0)
          temp_y = 0;

        if(using_min_only == 1 && (map[(int)min(pos_y,at_y)][(int)min(pos_x,at_x)] != 1 || map[(int)temp_y][(int)min(pos_x,at_x)] !=1))
        { 

        }
        else if(pos_x == (double)int_pos_x && pos_y == (double)int_pos_y && map[(int)min(pos_y,at_y)][(int)min(pos_x,at_x)] < map[(int)temp_y][(int)min(pos_x,at_x)])
        {
          // because we lack look ahead, there could be a problem if the grid on the other side of 
          // the edge (to which the back pointers end at) has a higher cost value than the one on 
          // the current grid. This is only a concern if (due to look ahead) pose is current at a corner
          // so if that happens, we don't use the gradient stuff.
        }
        else
        {
          // find interpolation between high_y and low_y
          double new_pos_x = low_x + ((at_x - (double)int_at_x)*(high_x - low_x));
          double new_pos_y = low_y;

          // the gradient method is also worse than the linear interpolation method if the resulting angle between the 
          // (gradient) back pointer and the closest axis is in between 23 and 28 degrees.
          //double delta_y = max(at_y - pos_y, pos_y - at_y);
          //double delta_x = max(at_x - pos_x, pos_x - at_x);
          double delta_y = max(at_y - new_pos_y, new_pos_y - at_y);
          double delta_x = max(at_x - new_pos_x, new_pos_x - at_x);
          double angle;
          if(delta_y == 0 || delta_x == 0)
            angle = 0;
          else if(delta_y > delta_x)
            angle = atan(delta_x/delta_y)/(PI)*180;
          else
            angle = atan(delta_y/delta_x)/(PI)*180;

          if(angle_min < angle && angle < angle_max && using_bad_range == 1)
          {
            // in the bad range
          }
          else
          {
            // reset pos_x based on interpolation between high_x and low_x
            pos_x = new_pos_x;
            pos_y = new_pos_y;

            // if this is in a grid that is left or right of where we started, we need to solve for the exit point of this grid
            if((int)pos_x != int_at_x) // && high_x != low_x)
            {
              int crossed_x;
              // printf("changed horiz grid \n");

              // find the x value of the vertical line that we crossed
              if(at_x > pos_x) // then we crossed int_at_x
                crossed_x = (double)int_at_x;
              else // pos_x > at_x  we crossed (int)pos_x = int_at_x + 1
                crossed_x = (double)int_at_x + 1;

              if(pos_y > at_y)
                pos_y = at_y + (crossed_x - at_x)/(pos_x - at_x);
              else
                pos_y = at_y - (crossed_x - at_x)/(pos_x - at_x);

              pos_x = crossed_x;
            }

            // readjust totalPathCost
            totalPathCost = totalPathCost - bestCellPath->g_to_edge + map[(int)min(pos_y, at_y)][(int)min(pos_x,at_x)]*sqrt((pos_x - at_x)*(pos_x - at_x) + (pos_y - at_y)*(pos_y - at_y)); 
            // printf("on horiz edge, gradient \n");
          }
        }
      }
    }

    robotsPath = addPathEnd(robotsPath, pos_y, pos_x); // this remembers where the robot has moved, used for graphics

    /*---------- start of graphics stuff ---------*/
    #ifndef NOGLUT
    if (DISPLAYGRAPHICS == 1)
    {
      clearDisplay();
      ////printMap(); // on command line
      ////printHeap();
      ////checkHeap();
      drawMap(-1,-.5);
      ////drawField(-1,-.5);
      drawHeapOnMaze(.5);
      drawGraphBackLinks(clrB, .6);

      drawPathOnMaze(robotsPath, clrW, .75); // draw the path

      //drawNodeBackLinks(&graph[int_at_y][int_at_x], clrR, .77);
      //if(at_x == (double)int_at_x && at_y != (double)int_at_y) // on a vertical line but not horizontal
      //  drawNodeBackLinks(&graph[int_at_y+1][int_at_x], clrR, .77);
      //else if(at_x != (double)int_at_x && at_y == (double)int_at_y) // on a horizontal line but not vertical
      //  drawNodeBackLinks(&graph[int_at_y][int_at_x+1], clrR, .77);

      ////printPath(robotsPath); // on command line
      flushDisplay();
      glutMainLoopEvent();
    }
    //getchar();
     #endif
    /*----------- end of graphics stuff ----------*/ 
    
    // re-initialize the cellPath look-ahead heap
    cpCleanHeap(primaryCellPathHeap);  
    deleteCellPath(bestCellPath);
  }  
  
  #ifdef R2GOAL
    robotsPath = addPathEnd(robotsPath, goalRN, goalRM); // add goal to the end of the path
    // add final segment cost
    double final_c = map[intMin2((int)goalRN, pos_y)][intMin2((int)goalRM, pos_x)];
    if(goalRN == pos_y && goalRM != pos_x && pos_y > 0)
    {
      if(map[intMin2((int)goalRN, pos_y)-1][intMin2((int)goalRM, pos_x)] < final_c)
         final_c = map[intMin2((int)goalRN, pos_y)-1][intMin2((int)goalRM, pos_x)];
    }
    else if(goalRM == pos_x && goalRN != pos_y && pos_x > 0)
    {
      if(map[intMin2((int)goalRN, pos_y)][intMin2((int)goalRM, pos_x)-1] < final_c)
         final_c = map[intMin2((int)goalRN, pos_y)][intMin2((int)goalRM, pos_x)-1];  
    }
   
    double final_d = sqrt((goalRN - pos_y)*(goalRN - pos_y) + (goalRM - pos_x)*(goalRM - pos_x));
    totalPathCost = totalPathCost + final_c*final_d;
    
    // because neither 1-step look ahead nor gradient method were used to get the last point (note this throws off the visualization path cost)
    int p;
    for(p = robotsPath->length-3; p >= 0; p--)
    {
      double pos_y_a = robotsPath->y[p];
      double pos_x_a = robotsPath->x[p];
      
      double pos_y_b = robotsPath->y[robotsPath->length-1];
      double pos_x_b = robotsPath->x[robotsPath->length-1];
        
      if((int)pos_y_b == (int)pos_y_a || (double)((int)pos_y_b + 1) == pos_y_a || (double)((int)pos_y_a + 1) == pos_y_b) // share same cell rows
      {
        if((int)pos_x_b == (int)pos_x_a || (double)((int)pos_x_b + 1) == pos_x_a || (double)((int)pos_x_a + 1) == pos_x_b) // share same cell column  
        {
          // because of triangle inequality, we can remove this point   

          robotsPath->x[p+1] = robotsPath->x[robotsPath->length-1];
          robotsPath->y[p+1] = robotsPath->y[robotsPath->length-1];
          
        }
        else
          break;
      }          
    }
    
  #endif
  
  robotsPath = removeRepeatedPoints(robotsPath);
  //printf("My total path cost = %f \n",totalPathCost);
  current_path_cost = totalPathCost; // store in a global (for visualization only)
  return robotsPath;
}

#ifndef MAINFILE

int main(int argc, char *argv[])
{
  // ---------- graphics stuff -- M.O. -------
  #ifndef  NOGLUT
  if (DISPLAYGRAPHICS == 1)
  {
    glutInit(&argc,argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutCreateWindow("Map");
    glutReshapeFunc(reshape);
    glEnable(GL_DEPTH_TEST);
  }  
  #endif
  // ---------- end of graphics stuff --------  
  
  int k, l;
  time_t rand_seed;
  (void) time(&rand_seed);
  srandom((long) rand_seed);
  //srandom(1226288753);
  fflush(stdout);  

  double costArray[] = 
               {1, 2, 1, 1, 1, 1, 9, 9, 9, 9,
                4, 1, 1, 1, 1, 1, 9, 9, 9, 9,
                10, 1, 1, 10, 1, 3, 9, 9, 9, 9,
                1, 1, 1, 1, 1, 1, 10, 9, 9, 1,
                1, 1, 9, 1, 4, 10, 9, 9, 1, 1,
                8, 1, 1, 1, 9, 10, 9, 1, 1, 1,
                7, 1, 1, 9, 9, 9, 1, 1, 1, 2,
                6, 1, 9, 9, 9, 1, 1, 4, 5, 3,
                1, 1, 9, 9, 1, 1, 1, 1, 1, 4,
                1, 1, 1, 1, 1, 1, 1, 1, 6, 5};

//   double costArray[] = 
//                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
//                 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
//                 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
//                 1, 1, 1, 1, 10, 1, 1, 1, 1, 1,
//                 1, 1, 1, 1, 10, 1, 1, 1, 1, 1,
//                 1, 1, 1, 1, 10, 1, 1, 1, 1, 1,
//                 1, 1, 1, 1, 10, 1, 1, 1, 1, 1,
//                 1, 1, 1, 1, 10, 1, 1, 1, 1, 1,
//                 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
//                 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};  
  
   buildGraphAndMap(50, 50);
    
   //newMapOfOnes();
   //newMapOfRandom(10, 1);
   //newMapOfBinary(100, .15);
   //newMapFractal(100000, .2,.6, 0);
   newMapFractal(10, .2,.6, 0);
   //newMapFromArray(costArray);
   //newMapFromArrayColumnMajor(costArray);
   //newMapFromBitmap("map1.bmp");
    
   /*---------- start of graphics stuff ---------*/ 
   #ifndef  NOGLUT
   if (DISPLAYGRAPHICS == 1)
   {
     clearDisplay();
     drawMap(-1,-.5);
     int g;
     for(g = 1; g < 1000; g++) // setting glut up lags
     {
       flushDisplay();
       glutMainLoopEvent();
     }
   }
   #endif
   /*----------- end of graphics stuff ----------*/ 
   
   // set goal and robot nodes (search goes goal -> robot)
   #ifdef R2GOAL // goal can be anywhere in R2
     goalRN = 18.5; 
     goalRM = 1.5;            
   #else
     //int goalN = row_with_min_val(WIDTH/6); //HEIGHT/2;
     int goalN = HEIGHT/2;
     int goalM = WIDTH/6;//3*WIDTH/4;// WIDTH/2; // 0
     s_goal = &graph[goalN][goalM];
   #endif
   int robotN = 5;
   int robotM = 40;
   s_start = &graph[robotN][robotM];
   
   // find the exact coordinates of the robot
   robot_pos_x = (double)robotM;
   robot_pos_y = (double)robotN;
   
   if(s_start == s_goal)
   {
     printf("Goal is already reached by the Robot \n");
     return;
   }
   
   /*---------------- start of robot movement code ----------------------*/
   buildHeap(); 
   
   s_start->g = LARGE;
   s_start->rhs = LARGE;

   #ifdef R2GOAL // goal can be anywhere in R2      

     // insert all nodes around the goal
     int N_low, N_high, M_low, M_high;
     if(goalRN == (double)((int)goalRN)) // GoalRN == GoalN
     {
       if(goalRM == (double)((int)goalRM)) // GoalRM == GoalM
       {
         // need to insert only one node
         N_low = (int)goalRN;
         N_high = (int)goalRN;
       
         M_low = (int)goalRM;
         M_high = (int)goalRM;
       }
       else
       {
         // need to insert 6 neighbors
         N_low = (int)goalRN-1;
         N_high = (int)goalRN+1;
       
         M_low = (int)goalRM;
         M_high = (int)goalRM + 1;    
       }    
     }
     else
     {
       if(goalRM == (float)((int)goalRM)) // GoalRM == GoalM
       {
         // need to insert 6 neighbors
         N_low = (int)goalRN;
         N_high = (int)goalRN + 1;
       
         M_low = (int)goalRM-1;
         M_high = (int)goalRM+1;  
       }
       else
       {
         // need to insert 4 neighbors
         N_low = (int)goalRN;
         N_high = (int)goalRN + 1;
       
         M_low = (int)goalRM;
         M_high = (int)goalRM + 1;  
       }
     } 
     
     // check bounds
     if(N_low < 0)
       N_low = 0;
     if(N_high < 0)
       N_high = 0;
      
     if(N_low >= HEIGHT)
       N_low = HEIGHT-1;
     if(N_high >= HEIGHT)
       N_high = HEIGHT-1;
   
     if(M_low < 0)
       M_low = 0;
     if(M_high < 0)
       M_high = 0;
     
     if(M_low >= WIDTH)
       M_low = WIDTH-1;
     if(M_high >= WIDTH)
       M_high = WIDTH-1;

     if(N_low == N_high && M_low == M_high) // just insert goal
     {
       s_goal = &graph[N_low][M_low];

       s_goal->g = LARGE;
       s_goal->rhs = 0;
     
       insert(s_goal,calculateKey(s_goal)); 
     }
     else // need to insert node neighbors
     {
       int n, m;
       for(n = N_low; n <= N_high; n++)
       {
         for(m = M_low; m <= M_high; m++)   
         {
           s_goal = &graph[n][m];
           
           double c = map[intMin2((int)goalRN, n)][intMin2((int)goalRM, m)];
           
           if(goalRN == n && n > 0) // possibility neighboring cell is better
           {
             if(map[n-1][intMin2((int)goalRM, m)] < c)
               c = map[n-1][intMin2((int)goalRM, m)];
           }
           else if(goalRM == m && m > 0) // possibility neighboring cell is better
           {
             if(map[intMin2((int)goalRN, n)][m-1] < c)
               c = map[intMin2((int)goalRN, n)][m-1];  
           }
           
           double d = sqrt((goalRN - (double)n)*(goalRN - (double)n) + (goalRM - (double)m)*(goalRM - (double)m));
           s_goal->g = LARGE;
           s_goal->rhs = c*d;
           insert(s_goal,calculateKey(s_goal)); 
         } 
       }
       s_goal = NULL;
     }
   
   #else
     s_goal->g = LARGE;
     s_goal->rhs = 0;
   
     insert(s_goal,calculateKey(s_goal));
   #endif
    
   MapPath* robotsPath = newMapPath(0); 
   robotsPath = addPathEnd(robotsPath, robot_pos_y, robot_pos_x); // this remembers where the robot has moved, used for graphics
     
   /*----------------- start of initial search loop ---------------------*/ 
   // compute the field costs
   computeShortestPath();
   
   // create the cellPath search and look ahead heaps
   primaryCellPathHeap = cpBuildHeap();
   secondaryCellPathHeap = cpBuildHeap();
      
   #ifndef R2GOAL
   // extract the path, this will be used to figure out where to move the robot     
   MapPath* pathToGoal = extractPath();  // in practice this can be changed to only return the first n points along the path
   double path1_cost = calculatePathCost(pathToGoal);

   MapPath* pathToGoalWithLookAhead = extractPathOneLookahead();
   double path2_cost = calculatePathCost(pathToGoalWithLookAhead);
   #endif
           
   MapPath* pathToGoalMine = extractPathMine(0);
   double path3_cost = calculatePathCost(pathToGoalMine);

   MapPath* pathToGoalMine2 = extractPathMine(1);
   double path4_cost = calculatePathCost(pathToGoalMine2);

   /*---------- start of graphics stuff ---------*/
   #ifndef  NOGLUT
   if (DISPLAYGRAPHICS == 1)
   {
     clearDisplay();
     //printMap(); // on command line
     //printHeap();
     //checkHeap();
     drawMap(-1,-.5);
     //drawField(-1,-.5);
     drawHeapOnMaze(.5);
     drawGraphBackLinks(clrB, .6);
     //printPath(pathToGoal); // on command line
     #ifndef R2GOAL
     drawPathOnMaze(pathToGoal, clrW, .75); // draw the path
     drawPathOnMaze(pathToGoalWithLookAhead, clrY, .76); // draw the path
     #endif         
     drawPathOnMaze(pathToGoalMine, clrR, .77); // draw the path
     drawPathOnMaze(pathToGoalMine2, clrG, .78); // draw the path 
     
     flushDisplay();
     glutMainLoopEvent();
   }
   #endif
   /*----------- end of graphics stuff ----------*/ 
   
   printf("done with initial search \n");
   //pauseFor(.2);
   //getchar(); 
   /*------------------- end of initial search loop ---------------------*/

    
   /* ------------- start of replanning / movement phase ----------------*/
   int i, lr, ud;
   #ifdef R2GOAL // goal can be anywhere in R2
   while(robot_pos_x != goalRM || robot_pos_y != goalRN)        
   #else
   while(robot_pos_x != (double)goalM || robot_pos_y != (double)goalN)
   #endif
   {   
     printf("seed: %d \n", (long) rand_seed); // in case there is an error, remember random seed
      
     // find the new exact coordinates of the robot
     
     #ifndef R2GOAL
     //robot_pos_x = pathToGoal->x[1];
     //robot_pos_y = pathToGoal->y[1]; 
     
     if(path2_cost <= path3_cost && path2_cost <= path4_cost)
     {
       robot_pos_x = pathToGoalWithLookAhead->x[1];
       robot_pos_y = pathToGoalWithLookAhead->y[1];
     }
     else if(path3_cost <= path2_cost && path3_cost <= path4_cost)
     {
       robot_pos_x = pathToGoalMine->x[1];
       robot_pos_y = pathToGoalMine->y[1];
     }
     else // path4_cost <= path2_cost && path4_cost <= path3_cost
     {
       robot_pos_x = pathToGoalMine2->x[1];
       robot_pos_y = pathToGoalMine2->y[1];
     }
     #else
       robot_pos_x = pathToGoalMine->x[1];
       robot_pos_y = pathToGoalMine->y[1];      
     #endif
     
     robotsPath = addPathEnd(robotsPath, robot_pos_y, robot_pos_x); // remembering robots path for graphics

     //printf("new robot: (%f, %f) \n",robot_pos_y, robot_pos_x);
     #ifndef R2GOAL
     deleteMapPath(pathToGoal);   
     deleteMapPath(pathToGoalWithLookAhead);
     #endif
     deleteMapPath(pathToGoalMine);
     deleteMapPath(pathToGoalMine2);
     
     // get list of cost updates
     //MapUpdateList* anUpdateList = getRandomUpdateList(20, 10);
     MapUpdateList* anUpdateList = getRandomUpdateListNear(50, 10, (int)robot_pos_y, (int)robot_pos_x, 5);
     //MapUpdateList* anUpdateList = getRandomUpdateListNear(200, 10, (int)robot_pos_y, (int)robot_pos_x, 20);
     //printUpdateList(anUpdateList);
     
     // if any edge costs changed -----------------------------------------
     // modify edge values
     for(i = 0; i < anUpdateList->length; i++)
     {
       updadeCell(anUpdateList->rows[i], anUpdateList->columns[i], anUpdateList->obstacle[i]);
       //checkHeap();
       //pauseFor(.2);
       //getchar();  
     }
        
     // do replanning, note that we need to make sure that all neighboring nodes of the cell(s) containing the robot are updated 
     if(robot_pos_x == (double)((int)robot_pos_x)) // then on a vertical edge, need to plan to nodes on cells to the left and right of the robot
      lr = -1;
     else //only need to plan to nodes on cells with x == int_pos_x
      lr = 0;
    
     while(lr < 2)
     { 
       if(robot_pos_y == (double)((int)robot_pos_y)) // then on a horizontal edge, need to plan to nodes on cells to the top and bottom of the robot
         ud = -1;
       else //only need to plan to nodes on cells with y == int_pos_y
         ud = 0;

       if((int)robot_pos_x + lr < 0 || (int)robot_pos_x + lr > WIDTH)
       {
          lr++;
          continue;
       }

       while(ud < 2)
       { 
         if((int)robot_pos_y + ud < 0 || (int)robot_pos_y + ud > HEIGHT)
         {
           ud++;
           continue;
         }

         s_start = &graph[(int)robot_pos_y + ud][(int)robot_pos_x + lr];
         
         printf("computing path \n");
         computeShortestPath();  
         printf("done computing path \n");
         ud++;  
       }
       lr++;
     }
     
     #ifndef R2GOAL
     // extract the path
     //printf("extracting basic \n");
     pathToGoal = extractPath();  // in practice this can be changed to only return the first n points along the path 

     //printf("extracting 1-step look ahead \n");
     pathToGoalWithLookAhead = extractPathOneLookahead();
     #endif
     
     
     //printf("extracting my path 1\n");
     pathToGoalMine = extractPathMine(0);

     //printf("extracting my path 2\n");
     pathToGoalMine2 = extractPathMine(1);
     
     /*---------- start of graphics stuff ---------*/
     #ifndef  NOGLUT
     if (DISPLAYGRAPHICS == 1)
     {
       clearDisplay();
       //printGraph(); // on command line
       //printHeap();
       //checkHeap();
       drawMap(-1,-.5);
       //drawField(-1,-.5);
       drawUpdateList(anUpdateList, .25);
       drawHeapOnMaze(.5);
       drawPathOnMaze(robotsPath, clrP, .75);
       drawGraphBackLinks(clrB, .6);
       //printPath(pathToGoal); // on command line
       //drawPathOnMaze(pathToGoal, clrW, .75); // draw the path
       //drawPathOnMaze(pathToGoalWithLookAhead, clrY, .76); // draw the path
       drawPathOnMaze(pathToGoalMine, clrR, .77); // draw the path
       drawPathOnMaze(pathToGoalMine2, clrK, .78); // draw the path
       flushDisplay();
       glutMainLoopEvent();
     }
     /*----------- end of graphics stuff ----------*/ 
     //pauseFor(.2);
     //getchar(); 
     
     deleteMapUpdateList(anUpdateList);  // memory management
   }
   #endif
   printf("Goal has been reached by the Robot \n");  
   /*------------------ end of robot movement code ----------------*/

   #ifndef R2GOAL
   deleteMapPath(pathToGoal); 
   deleteMapPath(pathToGoalWithLookAhead);
   #endif
           
   deleteMapPath(pathToGoalMine);
   deleteMapPath(pathToGoalMine2);
    
   #ifndef R2GOAL
   double path5_cost = calculatePathCost(robotsPath);


   double path2_reduction = (path1_cost - path2_cost)/path1_cost;
   double path3_reduction = (path1_cost - path3_cost)/path1_cost;
   double path4_reduction = (path1_cost - path4_cost)/path1_cost;
   double path5_reduction = (path1_cost - path5_cost)/path1_cost;



   printf("Path Costs: %f %f %f %f %f \n", path1_cost, path2_cost, path3_cost, path4_cost, path5_cost);
   printf("Path Cost reduction: 0 %f %f %f %f \n", path2_reduction, path3_reduction, path4_reduction, path5_reduction);
   
   #endif
   
   // only do this when totally done
   cpDeleteHeap(primaryCellPathHeap);
   cpDeleteHeap(secondaryCellPathHeap);
   deleteHeap();   
   deleteGraphAndMap();
   deleteMapPath(robotsPath);
}

#endif
