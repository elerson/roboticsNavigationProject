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
double max(double a, double b);

// returns the min of two values
double min(double a, double b);

// takes values ranging from 0 to mult and scales to 1 to 0
double scaleColor(double col, double mult);

#ifndef NOGLUT
// draws a rectangle with a different color on each corner
void drawRectangleMultiColor(double* pos, double* size, double* color1, double* color2, double* color3, double* color4);

// draws a rectangle
void drawRectangle(double* pos, double* size, double* color);

// draws the current map
// max_cost of -1 takes max cost to be the maximum map value
void drawMap(double max_cost, double z_height);

// draws the current field based on the g values of the graph
void drawField(double max_g_value, double z_height);

// draws the current heap (open, closed, closed and no neighbors back-link) 
// on top of the graph.
void drawHeapOnMaze(double z_height);

// draws a path on the maze at height z_height
void drawPathOnMaze(MapPath* path, double* color, double z_height);

// THE FOLLOWING HAS BEEN IMPLIMENTED IN THE MAIN FILE BECAUSE IT NOW
// REQUIRES FUNCTIONS IN THAT FILE
// // draws the graph back links on the maze at height z_height
// void drawGraphBackLinks(double* color, double z_height);

// draws squares around grids that are in the update list at height z_height
void drawUpdateList(MapUpdateList* updateList, double z_height);

#endif