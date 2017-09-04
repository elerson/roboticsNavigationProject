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
 *  field d* search graph and separate map for an N-graph structure, this 
 *  implements the headers in fdsgraph.h. 
 */

// allocates the initial memory for the graph and map and sets up the graph 
// structure. height and width refer to map, because graph is 1 more. this 
// must be called before any of the next four functions
void buildGraphAndMap(int height, int width)
{
  int x, y, d, newx, newy;

  HEIGHT = height;  // set globals
  WIDTH = width;    // set globals
  
  if (graph == NULL)
  {
	graph = (node**)calloc(height+1, sizeof(node*));
    
    for (y = 0; y < height+1; ++y)  
	  graph[y] = (node*)calloc(width+1, sizeof(node)); 
    
	for (y = 0; y < height+1; ++y)
    {    
      for (x = 0; x < width+1; ++x)
      {
        graph[y][x].g = LARGE;
        graph[y][x].h = 0;
        graph[y][x].f = LARGE;
        graph[y][x].rhs = LARGE;
        graph[y][x].k[0] = LARGE;
        graph[y][x].k[1] = LARGE;
		graph[y][x].x = x;
		graph[y][x].y = y;
        graph[y][x].bestNeighbor = NULL;
        graph[y][x].inHeap = false;
        graph[y][x].heapIndex = -1;
		for (d = 0; d < NEIGHBORS; ++d)
        {
		  newy = y + dy[d];
		  newx = x + dx[d];
          
          if(newy >= 0 && newy < height+1 && newx >= 0 && newx < width+1)
            graph[y][x].neighbors[d] = &graph[newy][newx];
          else
            graph[y][x].neighbors[d] = NULL;        
        }
	  }
    }  
  }
  
  if (map == NULL)
  {
	map = (double**)calloc(height, sizeof(double*));
    
    for (y = 0; y < height; ++y)  
	  map[y] = (double*)calloc(width, sizeof(double)); 
  }
}


// this populates a graph with ones
void newMapOfOnes()
{
  int x, y;
  for (y = 0; y < HEIGHT; ++y)
    for (x = 0; x < WIDTH; ++x)  
       map[y][x] = 1;  
} 

// this populates a graph with random values
void newMapOfRandom(double maxCost, double density)
{
  int x, y;
  for (y = 0; y < HEIGHT; ++y)
    for (x = 0; x < WIDTH; ++x)  
    {
      if(rand() % 10000 < 10000 * density)
        map[y][x] = 1 + (double)((rand() % (int)(1000*(maxCost-1)+1)))/1000;
      else
        map[y][x] = 1;  
    }
} 

// creates an N-graph where the costs of percentage density nodes are maxCost, the rest get values of 1
void newMapOfBinary(double maxCost, double density)
{
  int x, y;
  for (y = 0; y < HEIGHT; ++y)
    for (x = 0; x < WIDTH; ++x)  
    {
      if(rand() % 10000 < 10000 * density)
        map[y][x] = maxCost;
      else
        map[y][x] = 1;  
    }
} 

double rand_num() // gives a random number between 0 and 1 at the resolution of 1/1000
{return (double)((rand() % (1001)))/1000;}

int int_max(int a, int b) // returns the max
{
  if(a > b)
    return a;
  return b;
}

// creates an N-graph where the costs are determined by fractal map generation, values are between 1 and maxCost, 0 <= bump_factor <= 1 
// 0 <= uniformity <= 1, where closer to 1 means that the maps overall height is more similar, map_type: 0 = normal fractal, 1 = binary, 2 = 3 cost types
void newMapFractal(double maxCost, double bump_factor, double uniformity, int map_type)
{
  int side_length = 1;
  int num_its = 0;
  int s;
  int y,x;
  int cost_method_flag = map_type; //0 = normal fractal, 1 = binary, 2 = 3 cost types


  double rand_range = 1;

  // need side length to be n^2 + 1 >= max(LENGTH, WIDTH)
  while( side_length < int_max(HEIGHT,WIDTH) - 1)
  {
    side_length = side_length*2;
    num_its = num_its + 1;
  }
  s = side_length;
  side_length = side_length + 1;

  // create a sandbox to work in 
  double** fractal_map = (double**)calloc(side_length, sizeof(double*));
  for (y = 0; y <= s; ++y)  
  {
    fractal_map[y] = (double*)calloc(side_length, sizeof(double)); 
    for (x = 0; x <= s; ++x)
      fractal_map[y][x] = -500000; // flag value
  }

  //initialize corners
  fractal_map[0][0] = (.5 - rand_num())*2*rand_range;
  fractal_map[s][0] = (.5 - rand_num())*2*rand_range;
  fractal_map[s][s] = (.5 - rand_num())*2*rand_range;
  fractal_map[0][s] = (.5 - rand_num())*2*rand_range;

  // do iterative part
  int this_r;
  int this_c;
  double na, nb, nc, nd;
  int s_max, r, c;
  int same_count = 0;
  for(s_max = 2; s_max <= s; s_max = s_max*2)
  {
    if(same_count < (int)(uniformity*(double)num_its))
    {}
    else
      rand_range = rand_range*bump_factor;


    // x case, we want only the odd multiples
    for(r = 1; r < s_max; r = r + 2)
    {
      this_r = r*s/s_max;
      for(c = 1; c < s_max; c = c + 2)
      {
        this_c = c*s/s_max;

        // neighbors are s/s_max left and right and above and below
        if(same_count < (int)(uniformity*(double)num_its))
          fractal_map[this_r][this_c] = (.5 - rand_num())*2*rand_range;
        else
        {
          na = fractal_map[this_r-s/s_max][this_c-s/s_max];
          nb = fractal_map[this_r-s/s_max][this_c+s/s_max];
          nc = fractal_map[this_r+s/s_max][this_c-s/s_max];
          nd = fractal_map[this_r+s/s_max][this_c+s/s_max];
          fractal_map[this_r][this_c] = (na + nb + nc + nd)/4 + (.5 - rand_num())*2*rand_range;
        }
      }
    }

    // t case, we want all multiples
    for(r = 0; r <= s_max; r++)
    {
      this_r = r*s/s_max;
      for(c = 0; c <= s_max; c++)
      {
        this_c = c*s/s_max;

        // if this place in the map has a value already
        if(fractal_map[this_r][this_c] != -500000)
          continue;


        if(same_count < (int)(uniformity*(double)num_its))
          fractal_map[this_r][this_c] = (.5 - rand_num())*2*rand_range;
        else
        {
          double nbrs = 4;
          // neighbors are s/s_max left and right and above and below
          if(this_r-s/s_max < 0) // no neighbor above
          {
            na = 0; //fractal_map[this_r-s/s_max+s][this_c];
            nbrs--;
          }
          else
            na = fractal_map[this_r-s/s_max][this_c];

          if(this_r+s/s_max > s) // no neighbor below
          {
            nb = 0; //fractal_map[this_r+s/s_max-s][this_c];
            nbrs--;
          }
          else
            nb = fractal_map[this_r+s/s_max][this_c];

          if(this_c-s/s_max < 0) // no neighbor to the left
          {
            nc = 0; //fractal_map[this_r][this_c-s/s_max+s];
            nbrs--;
          }
          else
            nc = fractal_map[this_r][this_c-s/s_max];

          if(this_c+s/s_max > s) // no neighbor to the right
          {
            nd = 0; //fractal_map[this_r][this_c+s/s_max-s];
            nbrs--;
          }
          else
            nd = fractal_map[this_r][this_c+s/s_max];

          fractal_map[this_r][this_c] = (na + nb + nc + nd)/nbrs + (.5 - rand_num())*2*rand_range;
        }
      }
    }
    if(same_count < (int)(uniformity*(double)num_its))
      same_count++;
  }

  // read fractal_map into map
  double min_val = fractal_map[0][0];
  double max_val = fractal_map[0][0];

  for(r = 0; r < HEIGHT; r++)
  {
    for(c = 0; c < WIDTH; c++)
    {
      map[r][c] = fractal_map[r][c];

      if(map[r][c] < min_val)
        min_val = map[r][c];

      if(map[r][c] > max_val)
        max_val = map[r][c];
    }
  }

  // scale map to desired range 
  for(r = 0; r < HEIGHT; r++)
  {
    for(c = 0; c < WIDTH; c++)
    {

      if(cost_method_flag == 0) // normal fractal
        map[r][c] = 1+(map[r][c] - min_val)/(max_val - min_val)*(maxCost-1);
      else if(cost_method_flag == 1) // binary
      {
        map[r][c] = (map[r][c] - min_val)/(max_val - min_val)*maxCost;

        if(map[r][c] > .5*maxCost)
          map[r][c] = maxCost;
        else
          map[r][c] = 1;
      }
      else if(cost_method_flag == 2) // 3 cost types
      {
        map[r][c] = (map[r][c] - min_val)/(max_val - min_val)*maxCost;

        if(map[r][c] > .67*maxCost)
          map[r][c] = maxCost;
        else if(map[r][c] > .33*maxCost)
          map[r][c] = 5;
        else
          map[r][c] = 1;
      }
    }
  }

  // clean up
  for (y = 0; y <= s; ++y)
    free(fractal_map[y]);
  free(fractal_map);
}


// populates the map row-wise from a vector -- M.O.
void newMapFromArray(double* costArray)
{
  int x, y;
  for (y = 0; y < HEIGHT; ++y)
    for (x = 0; x < WIDTH; ++x)  
      map[y][x] = costArray[y*WIDTH + x];  
}  
      
// populates the map column-wise from a vector -- M.O.
void newMapFromArrayColumnMajor(double* costArray)
{
  int x, y;
  for (y = 0; y < HEIGHT; ++y)
    for (x = 0; x < WIDTH; ++x)  
      map[y][x] = costArray[y + x*HEIGHT];  
}


///////////////////////////////////// stuff for loading images

struct float_array;
typedef struct float_array float_array;


struct float_array
{
    int rows, cols; // size of A
    int temp;       // a flag to help with memory management
    int t;          // a flag to allow for easy transposing
    float** A;
};

float_array* make_float_array(int rows, int cols)
{
    float_array* this_array = (float_array*)calloc(1, sizeof(float_array));

    this_array->rows = rows;
    this_array->cols = cols;
    this_array->temp = 0;
    this_array->t = 0;

    this_array->A = (float**)calloc(rows, sizeof(float*));
    int i;
    for(i = 0; i < rows; i++)
        this_array->A[i] = (float*)calloc(cols, sizeof(float));
    return this_array;
}

// adjusts the array A randomly, where each column's max adjustment is given by +/- the corresponding 
// value in R_max. Note A itself is changed (a new array is not returned)
float_array* adjust_array_randomly(float_array* A, float* R_max)
{
  int i,j;
  float** AA = A->A;
  int rows = A->rows;
  int cols = A->cols;

  for(i = 0; i < rows; i++)
    for(j = 0; j < cols; j++)
      AA[i][j] += R_max[j]*(1 - 2*(float)(rand() % (int)(101))/100); // adds random number on [-R_max ... R_max]

  return A;
}

void destroy_float_array(float_array* this_array)
{
  if(this_array == NULL)
    return;

  int rows = this_array->rows;

  int i;
  for(i = 0; i < rows; i++)
    free(this_array->A[i]);

  free(this_array->A);
  free(this_array);
}

// prints the float array, also destroys it if it is a temp
void print_float_array(float_array* this_array)
{
  int i, j;
  if(this_array == NULL)
  {
    printf("0\n");
    return;
  }

  int rows = this_array->rows;
  int cols = this_array->cols;

  if(this_array->t == 0)
  {
    for(i = 0; i < rows; i++)
    {
      for(j = 0; j < cols; j++)
        printf("%f ",this_array->A[i][j]);
      printf("\n");
    }
  }
  else // it is inverted
  {
    for(j = 0; j < cols; j++)
    {
      for(i = 0; i < rows; i++)
        printf("%f ",this_array->A[i][j]);
      printf("\n");
    }
  }

  if(this_array->temp == 1)
    destroy_float_array(this_array);
}


#define TWOBYTE unsigned short
#define FOURBYTE unsigned long
#define LONG unsigned long

struct Bitmap;
typedef struct Bitmap Bitmap;

struct Image;
typedef struct Image Image;

struct Bitmap
{
  // BMP header stuff
  TWOBYTE BMP_Type;               // 2 bytes
  FOURBYTE BMP_Size;              // 4 bytes
  FOURBYTE BMP_Reserved;          // 4 bytes
  FOURBYTE BMP_Offset;            // 4 bytes

  // DIB header stuff
  FOURBYTE DBI_Size;              // 4 bytes
  LONG DBI_Width;                 // 4 bytes
  LONG DBI_Height;                // 4 bytes
  TWOBYTE DBI_Colorplanes;        // 2 bytes
  TWOBYTE DBI_BitsPerPixel;       // 2 bytes
  FOURBYTE DBI_CompressionMethod; // 4 bytes
  FOURBYTE DBI_ImageSize;         // 4 bytes
  LONG DBI_HorizPixelsPerMeter;   // 4 bytes
  LONG DBI_VertPixelsPerMeter;    // 4 bytes
  FOURBYTE DBI_PaletteSize;       // 4 bytes
  FOURBYTE DBI_ImportantColors;         // 4 bytes

  unsigned long BPP;
  unsigned long width;
  unsigned long height;
  unsigned long size;
  unsigned char* Bitmap_Image;
  unsigned char* palette;
  unsigned int bps;
  unsigned int CompressionFormat;
};

Bitmap* make_Bitmap()
{
  Bitmap* Bmp = (Bitmap*)calloc(1, sizeof(Bitmap));
  Bmp->BPP=0;
  Bmp->width=0;
  Bmp->height=0;
  Bmp->Bitmap_Image = NULL;
  Bmp->palette = NULL;
  Bmp->size=0;
  Bmp->bps=0;
  Bmp->CompressionFormat=0;

  return Bmp;
}

void destroy_Bitmap(Bitmap* Bmp)
{
  if(Bmp == NULL)
    return;

  free(Bmp->Bitmap_Image);

  if(Bmp->palette != NULL)
    free(Bmp->palette);

  free(Bmp);
}

void print_Bitmap_info(Bitmap* B)
{
  if( B == NULL)
	  return;

  printf("BMP Header: \n");
  printf(" Type: %x \n", B->BMP_Type);
  printf(" Size: %u \n", B->BMP_Size);
  printf(" Reserved: %u \n", B->BMP_Reserved);
  printf(" Offset: %u \n", B->BMP_Offset);

  printf("DIB Header: \n");
  printf(" Size: %u \n", B->DBI_Size);
  printf(" Width: %u \n", B->DBI_Width);
  printf(" Height: %u \n", B->DBI_Height);
  printf(" Color Planes: %u \n", B->DBI_Colorplanes);
  printf(" Bits Per Pixel: %u \n", B->DBI_BitsPerPixel);
  printf(" Compression Method: %u \n", B->DBI_CompressionMethod);
  printf(" Image Size: %u \n", B->DBI_ImageSize);
  printf(" Horizontal Pixels Per Meter: %u \n", B->DBI_HorizPixelsPerMeter);
  printf(" Vertical Pixels PEr Meter: %u \n", B->DBI_VertPixelsPerMeter);
  printf(" Palette Size: %u \n", B->DBI_PaletteSize);
  printf(" Important Colors: %u \n", B->DBI_ImportantColors);
}


Bitmap*  load_Bitmap_from_file(const char *filename)
{
  Bitmap* B = make_Bitmap();

  FILE* inf = NULL;
  unsigned int ImageIdx = 0;
  unsigned char* Bitmap_Image = NULL;

  if(!filename)
  {
    printf("can't open file");
    return NULL;
  }
  else
  {
    inf = fopen(filename,"rb");
    if(!inf)
    {
      printf("can't open file \n");
      return NULL;
    }
  }

  // read in the 14 byte BMP header
  fread(&B->BMP_Type,sizeof(TWOBYTE),1,inf);
  fread(&B->BMP_Size,sizeof(FOURBYTE),1,inf);
  fread(&B->BMP_Reserved,sizeof(FOURBYTE),1,inf);
  fread(&B->BMP_Offset,sizeof(FOURBYTE),1,inf);

  if(ferror(inf))
  {
    printf("problem with file header \n");
    fclose(inf);
    return NULL;
  }

  // read in the 40 byte DIB Header, assuming V3 is used
  fread(&B->DBI_Size,sizeof(FOURBYTE),1,inf);
  if(B->DBI_Size != 40)
  {
    printf("cannot open file because it does not use DBI header V3 \n");
    fclose(inf);
    return NULL;
  }
  fread(&B->DBI_Width,sizeof(LONG),1,inf);
  fread(&B->DBI_Height,sizeof(LONG),1,inf);
  fread(&B->DBI_Colorplanes,sizeof(TWOBYTE),1,inf);
  fread(&B->DBI_BitsPerPixel,sizeof(TWOBYTE),1,inf);
  fread(&B->DBI_CompressionMethod,sizeof(FOURBYTE),1,inf);
  fread(&B->DBI_ImageSize,sizeof(FOURBYTE),1,inf);
  fread(&B->DBI_HorizPixelsPerMeter,sizeof(LONG),1,inf);
  fread(&B->DBI_VertPixelsPerMeter,sizeof(LONG),1,inf);
  fread(&B->DBI_PaletteSize,sizeof(FOURBYTE),1,inf);
  fread(&B->DBI_ImportantColors,sizeof(FOURBYTE),1,inf);

  if(ferror(inf))
  {
    printf("problem with info header \n");
    fclose(inf);
    return NULL;
  }

  // extract color palette
  if(B->DBI_BitsPerPixel == 24)
  {
    // don't need to
  }
  else if(B->DBI_BitsPerPixel == 8 || B->DBI_BitsPerPixel == 4 || B->DBI_BitsPerPixel == 1)
  {

    int colors_used = B->DBI_PaletteSize;
    if(colors_used == 0)
      colors_used = (int)pow((float)2,(int)B->DBI_BitsPerPixel);

    //printf(" colors used: %d \n", colors_used);

    unsigned char* color_map = (unsigned char*)calloc(colors_used*4, sizeof(unsigned char));
    fread(color_map,sizeof(unsigned char),colors_used*4,inf);

    B->palette = color_map;
  }
  else
    printf("This number of bits per pixel (%u) not implemented \n", B->DBI_BitsPerPixel);


  fseek(inf,B->BMP_Offset,SEEK_SET);
  if(ferror(inf))
  {
    printf("problem with 'bfOffBits' \n");
    fclose(inf);
    return 0;
  }


  if(B->DBI_ImageSize != 0)
  {
    Bitmap_Image = (unsigned char*)calloc(B->DBI_ImageSize, sizeof(unsigned char));
    fread(Bitmap_Image,B->DBI_ImageSize,1,inf);
  }

  if(B->BMP_Type != 0x4D42)
  {
    printf("problem with magic number \n");
    fclose(inf);
  }

  if(!Bitmap_Image)
  {
    free(Bitmap_Image);
    fclose(inf);
  }

  if(Bitmap_Image==NULL)
    fclose(inf);

  B->Bitmap_Image = Bitmap_Image;

  fclose(inf);

  return B;
}

struct Image
{
  float_array* Red;
  float_array* Green;
  float_array* Blue;
};

Image* make_Image(int rows, int cols)
{
  Image* I = (Image*)calloc(1, sizeof(Image));
  I->Red = make_float_array(rows,cols);
  I->Green = make_float_array(rows,cols);
  I->Blue = make_float_array(rows,cols);

  return I;
}

void destroy_Image(Image* I)
{
  if(I == NULL)
    return;

  destroy_float_array(I->Red);
  destroy_float_array(I->Green);
  destroy_float_array(I->Blue);
  free(I);
}

void print_Image(Image* I)
{
  if(I == NULL)
    return;

  printf("Red: \n");
  print_float_array(I->Red);

  printf("Green: \n");
  print_float_array(I->Green);

  printf("Blue: \n");
  print_float_array(I->Blue);
}

// converts the Bitmap struct into an Image struct
Image* convert_Bitmap_to_double_array(Bitmap* B)
{
  if(B == NULL)
    return NULL;

  if(B->DBI_CompressionMethod != 0)
  {
    printf("unsupported compression type \n");
    return NULL;
  }

  int rows = B->DBI_Height;
  int cols = B->DBI_Width;

  Image* I = make_Image(rows, cols);
  float** Red = I->Red->A;
  float** Blue = I->Blue->A;
  float** Green = I->Green->A;
  unsigned char* Data = B->Bitmap_Image;

  int bits_per_pixel = B->DBI_BitsPerPixel;

  if(bits_per_pixel == 24)
  {
    int i, j, k;
    int col_size_with_pad = 3*cols;
    if(4*(col_size_with_pad/4) != col_size_with_pad)
      col_size_with_pad = 4*(col_size_with_pad/4+1);

    for(i = rows-1; i >= 0; i--)
    {
      k = (rows-i-1)*col_size_with_pad;
      for(j = 0; j < cols; j++)
      {
        Blue[i][j]  = (float)Data[k]/255;
        k++;
        Green[i][j] = (float)Data[k]/255;
        k++;
        Red[i][j] = (float)Data[k]/255;
        k++;
      }
    }
    return I;
  }
  else if(bits_per_pixel == 8)
  {
    int i, j, k;
    int col_size_with_pad = cols;
    if(4*(col_size_with_pad/4) != col_size_with_pad)
      col_size_with_pad = 4*(col_size_with_pad/4+1);

    for(i = rows-1; i >= 0; i--)
    {
      k = (rows-i-1)*col_size_with_pad;
      for(j = 0; j < cols; j++)
      {
        Blue[i][j]  = (float)B->palette[Data[k]*4]/255;
        Green[i][j] = (float)B->palette[Data[k]*4+1]/255;
        Red[i][j] = (float)B->palette[Data[k]*4+2]/255;
        k++;
      }
    }
    return I;

  }
  else if(bits_per_pixel == 4)
  {
    int i, j, k;
    int col_size_with_pad = cols;

    if(8*(col_size_with_pad/8) != col_size_with_pad)
      col_size_with_pad = 8*(col_size_with_pad/8+1);

    col_size_with_pad = col_size_with_pad/2;

    for(i = rows-1; i >= 0; i--)
    {
      k = (rows-i-1)*col_size_with_pad;
      unsigned char this_char = (unsigned char)Data[k];
      int bit_num = -1;
      int this_bit;

      for(j = 0; j < cols; j++)
      {
        bit_num++;
        if(bit_num == 2)
        {
          bit_num = 0;
          k++;
          this_char = Data[k];
        }
        this_bit = this_char/16;
        this_char = this_char << 4;


        Blue[i][j]  = (float)B->palette[this_bit*4]/255;
        Green[i][j] = (float)B->palette[this_bit*4+1]/255;
        Red[i][j] = (float)B->palette[this_bit*4+2]/255;
      }
    }
    return I;

  }
  else if(bits_per_pixel == 1)
  {
    int i, j, k;
    int col_size_with_pad = cols;

    if(32*(col_size_with_pad/32) != col_size_with_pad)
      col_size_with_pad = 32*(col_size_with_pad/32+1);

    col_size_with_pad = col_size_with_pad/8;

    for(i = rows-1; i >= 0; i--)
    {
      k = (rows-i-1)*col_size_with_pad;
      unsigned char this_char = (unsigned char)Data[k];
      int bit_num = -1;
      int this_bit;

      for(j = 0; j < cols; j++)
      {
        bit_num++;
        if(bit_num == 8)
        {
          bit_num = 0;
          k++;
          this_char = Data[k];
        }
        this_bit = this_char/128;
        this_char = this_char << 1;


        Blue[i][j]  = (float)B->palette[this_bit*4]/255;
        Green[i][j] = (float)B->palette[this_bit*4+1]/255;
        Red[i][j] = (float)B->palette[this_bit*4+2]/255;
      }
    }
    return I;

  }
  else
    printf("This number of bits per pixel not supported \n");

  return NULL;
}
////////// end stuff for loading images


// creates an N-graph where the costs are determined by the bitmap in filename
void newMapFromBitmap(char* filename)
{
  Bitmap* B = load_Bitmap_from_file(filename);
  Image* I = convert_Bitmap_to_double_array(B);

  int x, y;
  for (y = 0; y < HEIGHT; ++y)
    for (x = 0; x < WIDTH; ++x)  
      map[y][x] = 10001 - 10000*I->Blue->A[y][x];  

  destroy_Image(I);
  destroy_Bitmap(B);
}


// makes values in the map below (thresh_low thresh_high]
void thresholdMap(double thresh_low, double thresh_high, double val)
{
  int x, y;
  for(y = 0; y < HEIGHT; ++y)
    for(x = 0; x < WIDTH; ++x)  
      if(thresh_low < map[y][x] && map[y][x] <= thresh_high)
        map[y][x] = val;
}

// prints the entire graph on the command line
void printMap()
{
  int x, y;
  char ch[10];
  printf("_____");
  for (x = 0; x < WIDTH+2; ++x)  
    printf("___");  
  printf("\n");
  for (y = 0; y < HEIGHT; ++y)
  {
    printf("|");
	for (x = 0; x < WIDTH; ++x)
    {
      sprintf(ch, " %0.0f |", map[y][x]);  
      printf("%s", ch);
	}
    printf("\n");
  }
  printf("-----");
  for (x = 0; x < WIDTH+2; ++x)
    printf("---");
  printf("\n\n");
}

// this re-initializes the graph, saving cost values
void cleanGraphAndMap()
{
  int x, y;

  for (y = 0; y < HEIGHT+1; ++y)
  {    
    for (x = 0; x < WIDTH+1; ++x)
    {
      graph[y][x].g = LARGE;
      graph[y][x].h = 0;
      graph[y][x].f = LARGE;
      graph[y][x].rhs = LARGE;
      graph[y][x].k[0] = LARGE;
      graph[y][x].k[1] = LARGE;
	  graph[y][x].x = x;
	  graph[y][x].y = y;
      graph[y][x].bestNeighbor = NULL;
      graph[y][x].inHeap = false;
      graph[y][x].heapIndex = -1;
    }
  } 
  for (y = 0; y < HEIGHT; ++y)
  {    
    for (x = 0; x < WIDTH; ++x)
    {
      map[y][x] = 1;
    }
  }
}

// this takes care of memory management when the graph is deleted -- M.O.
void deleteGraphAndMap()
{
  int y;
  for (y = 0; y < HEIGHT+1; ++y)
    free(graph[y]);
  free(graph);  
  graph = NULL;
  
  for (y = 0; y < HEIGHT; ++y)
    free(map[y]);
  free(map);  
  map = NULL;
}

#ifdef CPP
// this takes care of memory management when the graph is deleted -- M.O.
// if delete_map == false, then the map is left intact
void deleteGraphAndMap(bool delete_map)
{
  int y;
  for (y = 0; y < HEIGHT+1; ++y)
    free(graph[y]);
  free(graph);  
  graph = NULL;
  
  if(delete_map)
  {
    for (y = 0; y < HEIGHT; ++y)
      free(map[y]);
    free(map);  
    map = NULL;
  }
}
#endif

// returns true if nodeA < nodeB based on key vectors [k[0] k[1]]
int nodeLess(node* nodeA, node* nodeB)
{
  if(nodeA->k[0] < nodeB->k[0])
      return true;
  else if(nodeA->k[0] == nodeB->k[0] && nodeA->k[1] < nodeB->k[1])
      return true;
  return false;
}

// returns true if nodeA <= nodeB based on key vectors [k[0] k[1]]
int nodeLesseq(node* nodeA, node* nodeB)
{
  if(nodeA->k[0] <= nodeB->k[0])
      return true;
  else if(nodeA->k[0] == nodeB->k[0] && nodeA->k[1] <= nodeB->k[1])
      return true;
  return false; 
}
