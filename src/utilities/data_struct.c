// data_struct.c - (c) Tyler Burdsall 2018
//
// Contains the function definitions from data_struct.h
#include "data_struct.h"

// Given two points, swap their values
void swap(point_xy* a, point_xy* b)
{
    point_xy temp;
    temp.x = a->x;
    temp.y = a->y;
    a->x = b->x;
    a->y = b->y;
    b->x = temp.x;
    b->y = temp.y;
}


// Returns the distance between two points
double
distance(const point_xy* a, const point_xy* b)
{
    double x = a->x - b->x;
    double y = a->y - b->y;
    x = x*x;
    y = y*y;
    return x + y;
}

// Given three points, determine if b is counter-clockwise to a
// and if c is counter-clockwise to b.
// The following return values correspond to a direction:
//
// 0 -> colinear
// 1 -> clockwise
// 2 -> counter-clockwise / one of the points is empty
int 
orientation(const point_xy* a, const point_xy* b, const point_xy* c)
{
    if (a == NULL || b == NULL || c == NULL)
    {
        return 2;
    }
    double val;
    val = (b->y - a->y) * (c->x - b->x) - (b->x - a->x) * (c->y - b->y);
    if (val == 0)
    {
        return 0;
    }
    if (val > 0)
    {
        return 1;
    }
    return 2;
}

// Compare function used to sort points by polar angle for
// the qsort function.
int
compare(const void* a, const void* b)
{
    point_xy* p1 = (point_xy*)a;
    point_xy* p2 = (point_xy*)b;

    int o = orientation(&p0, &*p1, &*p2);
    if (o == 0)
    {
        int dist = (distance(&p0, &*p2) >= distance(&p0, &*p1));
        if (dist == 1)
        {
            return -1;
        }
        return 1;
    }
    if (o == 2)
    {
        return -1;
    }
    return 1;
}
