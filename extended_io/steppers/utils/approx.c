#include <stdio.h>

typedef struct {
  int x;
  int y;
}  POINT;

#define TOTAL_POINTS 9

POINT points[] = 
{
{0,	      0},
{10,    260},
{20,    500},
{50,    730},
{100,   970},
{150,  1180},
{250,  1420},
{300,  1620},
{400,  1870},
};

int main()
{
    float slope;
    int delta;
    float y;

    printf("\n\
int flaps_x_to_y(int x)\n\
{\n\
    const int table[] = {\n    ");
    
    int i,j,k = 0;
    for(i=0; i<(TOTAL_POINTS-1); i++)
    {
        slope = (1.0*(points[i+1].y - points[i].y)) /  (1.0*(points[i+1].x - points[i].x));
        delta = (points[i+1].x - points[i].x);

        for(j=0;j<delta;j++)
        {
          y = 1.0 * points[i].y + slope * j;
          printf("%4.0f, ",y);
          if(! (++k%10))
          printf("\n    ");
        }
    }
    printf("%4.0f };\n\n",1.0*points[i].y);
    printf("\
    if(x < %d) x = %d;\n\
    if(x > %d ) x = %d;\n\
    return table[x];\n\
}\n", points[0].x, points[0].x, points[TOTAL_POINTS-1].x, points[TOTAL_POINTS-1].x);
    
    
}

