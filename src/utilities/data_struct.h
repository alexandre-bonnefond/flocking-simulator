// data_struct.h - (c) Tyler Burdsall 2018
// 
// Contains the struct definitions for a point and node as well
// as the function declarations for operations on a point.
#ifndef DATA_STRUCT
#define DATA_STRUCT
#include <stdlib.h>
#include <stdio.h>

typedef struct point_xy
{
    double x; 
    double y;
} point_xy;

typedef struct node
{
    point_xy* data;
    struct node* next;
} node;

point_xy  p0; // Global variable that will be used for the compare function
void   swap(point_xy*, point_xy*);
double distance (const point_xy*, const point_xy*);
int    orientation(const point_xy*, const point_xy*, const point_xy*);
int    compare(const void*, const void*);

typedef struct measurement_bundle {
    double currentAvg;
    double currentObstAvg;
    long count;
    long countObst;
} measurement_bundle;

measurement_bundle*** allocMeasurementMatrix(int agentCount, int ResolutionX, int ResolutionY, double initValue);

void freeMeasurementMatrix(measurement_bundle ***tmat, int agentCount, int ResolutionX, int ResolutionY);


enum MTYPES_ENUM {
    MTYPE_OBST,
    MTYPE_TRAIL
};

typedef enum MTYPES_ENUM MeasurementType;

void insertMeasurementIntoBundle(measurement_bundle ** bundle_mat, int x, int y, double measurement, MeasurementType type);

#endif
