//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/* Drawing copters, etc. (dynamics-specific objects) */

#include "dynspecviz.h"
#include "algo_spp_evol.h"
#include "utilities/obstacles.h"
#include <stdlib.h>

#define MIN(a,b) (a>b?b:a)

static double EyeFromCenter[3];
static double DistanceFromCenter;
static double NewEyeZ;

/* Draws velocity arrow above the copter */
void DrawVelocityArrow_2D(const double x, const double y,
        const double velocity_x, const double velocity_y,
        const double MapSizexy, const float *arrowcolor) {

    /* Drawing arrow... */
    DrawArrow(RealToGlCoord_2D(x, MapSizexy),
            RealToGlCoord_2D(y, MapSizexy),
            RealToGlCoord_2D(20.0 * pow(-50.0 + sqrt(velocity_x * velocity_x +
                                    velocity_y * velocity_y), 1.0 / 3.0),
                    MapSizexy), atan2(-velocity_y, velocity_x), arrowcolor);

}

/* Draws...
 * - a circle (with radius=40cm) as a representation of a quadcopter
 * - "Dangerous area" - a circle (with radius=1.5m)
 */
void DrawCopter_2D(const double x, const double y, const double MapSizexy,
        const double Radius, const float *agentcolor) {
    /* Real size of the copter */
    DrawFullCircle(RealToGlCoord_2D(x, MapSizexy), RealToGlCoord_2D(y,
                    MapSizexy), RealToGlCoord_2D(40, MapSizexy), agentcolor);
    /* Dangerous zone */
    DrawCircle(RealToGlCoord_2D(x, MapSizexy), RealToGlCoord_2D(y, MapSizexy),
            RealToGlCoord_2D(Radius / 2.0, MapSizexy), agentcolor);

}

/* Labels above agents */
void DrawAgentLabel_2D(phase_t * Phase, const int WhichAgent, char *Label,
        bool Display, vizmode_params_t * VizParams, const float *Color) {

    double *Position;
    Position = Phase->Coordinates[WhichAgent];

    if (Display == true) {

        DrawString(RealToGlCoord_2D(Position[0] - VizParams->CenterX + 120.0,
                        VizParams->MapSizeXY),
                RealToGlCoord_2D(Position[1] - VizParams->CenterY + 120.0,
                        VizParams->MapSizeXY), GLUT_BITMAP_TIMES_ROMAN_10,
                Label, Color);

    }

}

/* Draws a network arrow between two agents */
void DrawNetworkArrow_2D(phase_t * Phase, const int FromWhichAgent,
        const int ToWhichAgent, vizmode_params_t * VizParams,
        const float *color) {

    double *FromCoords;
    double *ToCoords;
    FromCoords = Phase->Coordinates[FromWhichAgent];
    ToCoords = Phase->Coordinates[ToWhichAgent];

    static double DifferenceVector[3];
    VectDifference(DifferenceVector, ToCoords, FromCoords);

    static double ArrowCenterX;
    ArrowCenterX = (FromCoords[0] + ToCoords[0]) * 0.5 - VizParams->CenterX;
    static double ArrowCenterY;
    ArrowCenterY = (FromCoords[1] + ToCoords[1]) * 0.5 - VizParams->CenterY;

    static double angle;
    DifferenceVector[2] = 0;
    angle = acos(DifferenceVector[0] / VectAbs(DifferenceVector));
    if (DifferenceVector[1] > 0) {
        angle *= -1;
    }

    DrawThinArrow(RealToGlCoord_2D(ArrowCenterX, VizParams->MapSizeXY),
            RealToGlCoord_2D(ArrowCenterY, VizParams->MapSizeXY),
            RealToGlCoord_2D(VectAbs(DifferenceVector) - 60,
                    VizParams->MapSizeXY), RealToGlCoord_2D(30,
                    VizParams->MapSizeXY), angle, color);

}

/* Draws a network arrow between two positions */
void DrawNetworkArrowBetweenPositions_2D(double *FromCoords, double *ToCoords,
        vizmode_params_t * VizParams, const float *color) {

    static double DifferenceVector[3];
    VectDifference(DifferenceVector, ToCoords, FromCoords);

    static double ArrowCenterX;
    ArrowCenterX = (FromCoords[0] + ToCoords[0]) * 0.5 - VizParams->CenterX;
    static double ArrowCenterY;
    ArrowCenterY = (FromCoords[1] + ToCoords[1]) * 0.5 - VizParams->CenterY;

    static double angle;
    DifferenceVector[2] = 0;
    angle = acos(DifferenceVector[0] / VectAbs(DifferenceVector));
    if (DifferenceVector[1] > 0) {
        angle *= -1;
    }

    DrawThinArrow(RealToGlCoord_2D(ArrowCenterX, VizParams->MapSizeXY),
            RealToGlCoord_2D(ArrowCenterY, VizParams->MapSizeXY),
            RealToGlCoord_2D(VectAbs(DifferenceVector) - 60,
                    VizParams->MapSizeXY), RealToGlCoord_2D(30,
                    VizParams->MapSizeXY), angle, color);

}

/* Drawing sensor range network */
void DrawSensorRangeNetwork_2D(phase_t * PhaseData,
        unit_model_params_t * Unit_params,
        const int WhichAgent, double ** Polygons,
        const int Now,
        vizmode_params_t * VizParams, const float *color) {

        int i, j;

        float Red[3];
        Red[0] = .9; Red[1] = 0.1; Red[2] = .1;
        float * RedColor;
        RedColor = Red;
        double *ActualAgentsCoordinates;
        ActualAgentsCoordinates = PhaseData[Now].Coordinates[WhichAgent];
        GetAgentsCoordinatesFromTimeLine(ActualAgentsCoordinates, PhaseData,
                WhichAgent, Now);
        bool DrawCircles = false;
        if (DrawCircles == true){
                DrawCircle(RealToGlCoord_2D(ActualAgentsCoordinates[0] - VizParams->CenterX , VizParams->MapSizeXY),
                RealToGlCoord_2D(ActualAgentsCoordinates[1] - VizParams->CenterY, 
                VizParams->MapSizeXY), RealToGlCoord_2D(Unit_params->R_C.Value, VizParams->MapSizeXY), RedColor);
        }
        static double NeighboursCoordinates[3];
        static double DifferenceVector[3];
        static double AbsDistance;

        static double ArrowCenterX;
        static double ArrowCenterY;
        static double angle;
        static double CenterX1, CenterX2, CenterY1, CenterY2; 

        //création du dégradé de couleurs
        float couleur [3] = {0,1,0};
        float green2 [3]= {0,1,0};
        float yellow2[3] ={1,1,0};
        float red2 [3] = {1,0,0};
        int HowManySteps = 8;
        float degrade [HowManySteps][3];

        /*for (int m = 0; m < HowManySteps; m++) {
                degrade [m][0] = 0;
                degrade [m][1] = 0;
                degrade [m][2] = 0;
        }*/

        //parcourir Laplacian pour trouver min et max
        //diviser le produit en croix par la diff entre min et max (caster en int)
        //prendre la valeur absolue toujurs, parce que les plus petites valeurs commencent au vert entre autre


        /*for (int n=0; n< HowManySteps; n++){
                for(int o=0; o<3; o++){
                        fprintf(stdout, "dégradé[%d][%d] = %f \n",n,o, degrade[n][o]);
                }
        }

        degrade [0][0] =  couleur[0];
        degrade [0][1] = couleur[1];
        degrade [0][2] =  couleur[2];
        for (int m = 0; m < HowManySteps; m++) {
                couleur[0] += (red2[0] - green2[0]) / HowManySteps;
                couleur[1] += (red2[1] - green2[1]) / HowManySteps;
                couleur[2] += (red2[2] - green2[2]) / HowManySteps;
                if (m>0){ 
                        degrade [m][0] = couleur[0];
                        degrade [m][1] = couleur[1];
                        degrade [m][2] = couleur[2];
                } 
        }*/

        //Création matrice Laplacienne symétrique
        float temp [PhaseData[0].NumberOfAgents][PhaseData[0].NumberOfAgents];

//Minimum
        for (int i = 0; i< 10; i++){
            for (int j=0; j<10; j++){
                if ((PhaseData[Now].Laplacian[i][j]>-70) && (PhaseData[Now].Laplacian[j][i]>-70)){
                    temp[i][j]= MIN(PhaseData[Now].Laplacian[i][j], PhaseData[Now].Laplacian[j][i]);
                } else if ((PhaseData[Now].Laplacian[i][j]>-70) && (PhaseData[Now].Laplacian[j][i]<-70)){
                    temp[i][j]= PhaseData[Now].Laplacian [i][j];
                } else  {
                    temp[i][j]= PhaseData[Now].Laplacian [j][i]; // soit ya que la valeur de ji qui est affichable soit aucune des 2 donc ça revient au même
                }
                fprintf (stdout, " %f ,", temp[i][j]);
            }
            fprintf (stdout, " \n");
        }

//Moyenne
/*        for (int i = 0; i< 10; i++){
            for (int j=0; j<10; j++){
                if ((PhaseData[Now].Laplacian[i][j]>-70) && (PhaseData[Now].Laplacian[j][i]>-70)){

                    temp[i][j]= ((PhaseData[Now].Laplacian[i][j] + PhaseData[Now].Laplacian[j][i])/2);

                } else if ((PhaseData[Now].Laplacian[i][j]>-70) && (PhaseData[Now].Laplacian[j][i]<-70)){

                    temp[i][j]= PhaseData[Now].Laplacian [i][j];

                } else  {

                    temp[i][j]= PhaseData[Now].Laplacian [j][i]; // soit ya que la valeur de ji qui est affichable soit aucune des 2 donc ça revient au même
                }
                fprintf (stdout, " %f ,", temp[i][j]);
            }
            fprintf (stdout, " \n");
        }
*/


        degrade [0][0] =  couleur[0];
        degrade [0][1] = couleur[1];
        degrade [0][2] =  couleur[2];
        for (int m = 0; m < (HowManySteps/2); m++) {
                couleur[0] += 2*(yellow2[0] - green2[0]) / HowManySteps;
                couleur[1] += 2*(yellow2[1] - green2[1]) / HowManySteps;
                couleur[2] += 2*(yellow2[2] - green2[2]) / HowManySteps;
                if (m > 0){
                        degrade [m][0] =  couleur[0];
                        degrade [m][1] = couleur[1];
                        degrade [m][2] =  couleur[2];
                }
        } 
        for (int k = (HowManySteps/2); k < HowManySteps; k++) {
                couleur[0] += 2*(red2[0] - yellow2[0]) / HowManySteps;
                couleur[1] += 2*(red2[1] - yellow2[1]) / HowManySteps;
                couleur[2] += 2*(red2[2] - yellow2[2]) / HowManySteps;
                degrade [k][0] =  couleur[0];
                degrade [k][1] = couleur[1];
                degrade [k][2] =  couleur[2];
        } 
        

        for (i = 0; i < PhaseData[0].NumberOfAgents; i++) {

                if (i != WhichAgent) {

                        GetAgentsCoordinatesFromTimeLine(NeighboursCoordinates, PhaseData,
                                i, Now);
                        VectDifference(DifferenceVector, ActualAgentsCoordinates,
                                NeighboursCoordinates);
                        AbsDistance = VectAbs(DifferenceVector);
                        
                        double *ToSort;
                        ToSort = malloc(sizeof(double) * PhaseData[0].NumberOfAgents);
                        for (int k = 0; k < PhaseData[0].NumberOfAgents; k++) {
                                ToSort[k] = PhaseData[Now].Laplacian[WhichAgent][k];
                        }

                        int *Indexes;
                        Indexes = malloc(sizeof(int) * PhaseData[0].NumberOfAgents);
                        
                        ArgMaxSort(ToSort, PhaseData[0].NumberOfAgents, Indexes);

                        bool IsLeading = false;
                        for (int l = 0; l < Size_Neighbourhood; l++) {
                                if (i == Indexes[l]) {
                                        IsLeading = true;
                                }
                        }
                        free(Indexes);
                        free(ToSort);

                        if ((int)(Unit_params->communication_type.Value) == 0) {

                                if (AbsDistance <= Unit_params->R_C.Value && IsLeading == true) {
                                        
                                        ArrowCenterX =
                                                (ActualAgentsCoordinates[0] +
                                                NeighboursCoordinates[0]) * 0.5 - VizParams->CenterX;
                                        ArrowCenterY =
                                                (ActualAgentsCoordinates[1] +
                                                NeighboursCoordinates[1]) * 0.5 - VizParams->CenterY;

                                        DifferenceVector[2] = 0;
                                        angle = -atan2(DifferenceVector[1], DifferenceVector[0]);

                                        DrawThinArrow(RealToGlCoord_2D(ArrowCenterX,VizParams->MapSizeXY),
                                                RealToGlCoord_2D(ArrowCenterY, VizParams->MapSizeXY),
                                                RealToGlCoord_2D(VectAbs(DifferenceVector) - 60,
                                                        VizParams->MapSizeXY), RealToGlCoord_2D(30,
                                                        VizParams->MapSizeXY), angle, color);
                                }

                                else if (AbsDistance <= Unit_params->R_C.Value) {

                                        CenterX1 = ActualAgentsCoordinates[0] - VizParams->CenterX;
                                        CenterY1 = ActualAgentsCoordinates[1] - VizParams->CenterY;

                                        CenterX2 = NeighboursCoordinates[0] - VizParams->CenterX;
                                        CenterY2 = NeighboursCoordinates[1] - VizParams->CenterY;

                                        DrawDashedLine(RealToGlCoord_2D(CenterX1,VizParams->MapSizeXY),
                                                RealToGlCoord_2D(CenterY1, VizParams->MapSizeXY),
                                                RealToGlCoord_2D(CenterX2,VizParams->MapSizeXY),
                                                RealToGlCoord_2D(CenterY2, VizParams->MapSizeXY),
                                                color);


                                }
                        }

                        else if ((int)(Unit_params->communication_type.Value) == 1) {

                                if (PhaseData[Now].Laplacian[WhichAgent][i] > Unit_params->sensitivity_thresh.Value && IsLeading == true) {
                                        
                                        ArrowCenterX =
                                                (ActualAgentsCoordinates[0] +
                                                NeighboursCoordinates[0]) * 0.5 - VizParams->CenterX;
                                        ArrowCenterY =
                                                (ActualAgentsCoordinates[1] +
                                                NeighboursCoordinates[1]) * 0.5 - VizParams->CenterY;

                                        DifferenceVector[2] = 0;
                                        angle = -atan2(DifferenceVector[1], DifferenceVector[0]);

                                        DrawThinArrow(RealToGlCoord_2D(ArrowCenterX,VizParams->MapSizeXY),
                                                RealToGlCoord_2D(ArrowCenterY, VizParams->MapSizeXY),
                                                RealToGlCoord_2D(VectAbs(DifferenceVector) - 60,
                                                        VizParams->MapSizeXY), RealToGlCoord_2D(30,
                                                        VizParams->MapSizeXY), angle, color);
                                }

                                else if (PhaseData[Now].Laplacian[WhichAgent][i] > Unit_params->sensitivity_thresh.Value) {

                                        CenterX1 = ActualAgentsCoordinates[0] - VizParams->CenterX;
                                        CenterY1 = ActualAgentsCoordinates[1] - VizParams->CenterY;

                                        CenterX2 = NeighboursCoordinates[0] - VizParams->CenterX;
                                        CenterY2 = NeighboursCoordinates[1] - VizParams->CenterY;

                                        DrawDashedLine(RealToGlCoord_2D(CenterX1,VizParams->MapSizeXY),
                                                RealToGlCoord_2D(CenterY1, VizParams->MapSizeXY),
                                                RealToGlCoord_2D(CenterX2,VizParams->MapSizeXY),
                                                RealToGlCoord_2D(CenterY2, VizParams->MapSizeXY),
                                                color);

                                }
                        }

                        else if ((int)(Unit_params->communication_type.Value) == 2) { // comm avec pertes dans les obstacles

                                if (PhaseData[Now].Laplacian[WhichAgent][i] > Unit_params->sensitivity_thresh.Value && IsLeading == true) {

                                        fprintf(stdout,"puissance %f, agent %d \n", temp[WhichAgent][i], WhichAgent);
                                        //val abs pour que ça soit de 50 (rouge) à 0(vert, état initial du tableau)

                                        //Première solution à la non symétrie : une moyenne des 2
                                        //float moyenne = (PhaseData[Now].Laplacian[WhichAgent][i]+PhaseData[Now].Laplacian[i][WhichAgent])/2;
                                        //int indice = (int) ((log10(abs(moyenne))-1.6)*(HowManySteps-1)/0.24); //produit en croix, on trouve un int pour savoir à quelle couleur la com correspond
                                        
                                        //Deuxième solution : on ne garde que la moins bonne
                                        /*int indice;
                                        if (PhaseData[Now].Laplacian[WhichAgent][i] < PhaseData[Now].Laplacian[i][WhichAgent]){
                                                indice = (int) ((log10(abs(PhaseData[Now].Laplacian[WhichAgent][i]))-1.6)*(HowManySteps-1)/0.24);
                                        } else {
                                                indice = (int) ((log10(abs(PhaseData[Now].Laplacian[i][WhichAgent]))-1.6)*(HowManySteps-1)/0.24);
                                        } */

                                        //int indice = (int) ((log10(abs(temp[WhichAgent][i]))-1.6)*(HowManySteps-1)/0.24); //produit en croix, on trouve un int pour savoir à quelle couleur la com correspond
                                        
                                        // tentative échelle log 
                                        float res = ((((abs(temp[WhichAgent][i])-40)*(100-10))/(70-40))+10);
                                        int indice = (int)(((log10(res)-1)*(7))/(2-1));

                                        //Pour éviter les erreurs de segmentation
                                        if (indice < 0 ){
                                                indice = 0;
                                        } else if (indice >7){
                                                indice = 7;
                                        }


                                        //color = degrade[indice];
                                        if (indice >= HowManySteps){
                                                indice = HowManySteps - 1;
                                        }
                                        
                                        ArrowCenterX =
                                                (ActualAgentsCoordinates[0] +
                                                NeighboursCoordinates[0]) * 0.5 - VizParams->CenterX;
                                        ArrowCenterY =
                                                (ActualAgentsCoordinates[1] +
                                                NeighboursCoordinates[1]) * 0.5 - VizParams->CenterY;

                                        DifferenceVector[2] = 0;
                                        angle = -atan2(DifferenceVector[1], DifferenceVector[0]);

                                        //dans cette boucle c'est que c'est au dessus de -70 normalement du coup ici appeler notre fonction de choix de couleur

                                        DrawThinArrow(RealToGlCoord_2D(ArrowCenterX,VizParams->MapSizeXY), //travailelr sur cette fonction sur l'input color
                                                RealToGlCoord_2D(ArrowCenterY, VizParams->MapSizeXY),
                                                RealToGlCoord_2D(VectAbs(DifferenceVector) - 60,
                                                        VizParams->MapSizeXY), RealToGlCoord_2D(30,
                                                        VizParams->MapSizeXY), angle, degrade[indice]); //degrade[indice]

                                        for (j = 0; j < obstacles.o_count; j++){

                                                double **Intersections;
                                                Intersections = malloc(sizeof(double *) * 2);
                                                Intersections[0] = malloc(sizeof(double) * 3);
                                                Intersections[1] = malloc(sizeof(double) * 3);

                                                int NumberOfIntersections;

                                                NumberOfIntersections = IntersectionOfSegmentAndPolygon2D(Intersections,
                                                PhaseData[Now].Coordinates[WhichAgent], PhaseData[Now].Coordinates[i], 
                                                Polygons[j], obstacles.o[j].p_count);

                                                if (NumberOfIntersections == 2){

                                                        DrawCircle(RealToGlCoord_2D(Intersections[0][0] - VizParams->CenterX, 
                                                        VizParams->MapSizeXY), RealToGlCoord_2D(Intersections[0][1] - VizParams->CenterY, 
                                                        VizParams->MapSizeXY), RealToGlCoord_2D(1000, VizParams->MapSizeXY), RedColor);

                                                        DrawCircle(RealToGlCoord_2D(Intersections[1][0] - VizParams->CenterX, 
                                                        VizParams->MapSizeXY), RealToGlCoord_2D(Intersections[1][1] - VizParams->CenterY, 
                                                        VizParams->MapSizeXY), RealToGlCoord_2D(1000, VizParams->MapSizeXY), RedColor);

                                                        // DrawLine(RealToGlCoord_2D(Intersections[0][0] - VizParams->CenterX, VizParams->MapSizeXY),
                                                        // RealToGlCoord_2D(Intersections[0][1] - VizParams->CenterY, VizParams->MapSizeXY),
                                                        // RealToGlCoord_2D(Intersections[1][0] - VizParams->CenterX, VizParams->MapSizeXY),
                                                        // RealToGlCoord_2D(Intersections[1][1] - VizParams->CenterY, VizParams->MapSizeXY), 
                                                        // RealToGlCoord_2D(80, VizParams->MapSizeXY), RedColor);
                                                }

                                                freeMatrix(Intersections, 2, 3);
                                        }
                                
                                }

                                /*else if (PhaseData[Now].Laplacian[WhichAgent][i] > Unit_params->sensitivity_thresh.Value) {

                                        //faut que le pointillé soit pas en blanc du coup
                                        int indice = (abs(PhaseData[Now].Laplacian[WhichAgent][i] + 40)*HowManySteps)/50;

                                        CenterX1 = ActualAgentsCoordinates[0] - VizParams->CenterX;
                                        CenterY1 = ActualAgentsCoordinates[1] - VizParams->CenterY;

                                        CenterX2 = NeighboursCoordinates[0] - VizParams->CenterX;
                                        CenterY2 = NeighboursCoordinates[1] - VizParams->CenterY;

                                        DrawDashedLine(RealToGlCoord_2D(CenterX1,VizParams->MapSizeXY),
                                                RealToGlCoord_2D(CenterY1, VizParams->MapSizeXY),
                                                RealToGlCoord_2D(CenterX2,VizParams->MapSizeXY),
                                                RealToGlCoord_2D(CenterY2, VizParams->MapSizeXY),
                                                degrade[indice]);

                                }*/

                        }

                }              

        }

}

/* 3D objects */

/* Drawing sensor range network */
void DrawSensorRangeNetwork_3D(phase_t * PhaseData,
        const double SensorRangeToDisplay,
        const double PowerThreshold,
        const int WhichAgent,
        const double Delay,
        const int Now,
        const double h, const double MapSizexy, const float *color) {

    int i;

    double *ActualAgentsCoordinates;
    ActualAgentsCoordinates = PhaseData[Now].Coordinates[WhichAgent];
    GetAgentsCoordinatesFromTimeLine(ActualAgentsCoordinates, PhaseData,
            WhichAgent, Now);

    static double NeighboursCoordinates[3];
    NullVect(NeighboursCoordinates, 3);
    static double DifferenceVector[3];
    NullVect(DifferenceVector, 3);

    for (i = 0; i < PhaseData[0].NumberOfAgents; i++) {
        if (i != WhichAgent) {

            GetAgentsCoordinatesFromTimeLine(NeighboursCoordinates, PhaseData,
                    i, Now);
            VectDifference(DifferenceVector, ActualAgentsCoordinates,
                    NeighboursCoordinates);
        //     if (VectAbs(DifferenceVector) < SensorRangeToDisplay) {
            if (VectAbs(DifferenceVector) < SensorRangeToDisplay &&
                        PhaseData[Now].Laplacian[WhichAgent][i] >= PowerThreshold) {
                
                glColor3f(color[0], color[1], color[2]);

                glBegin(GL_LINES);

                glVertex3f(RealToGlCoord_3D(ActualAgentsCoordinates[0],
                                MapSizexy),
                        RealToGlCoord_3D(ActualAgentsCoordinates[1], MapSizexy),
                        RealToGlCoord_3D(ActualAgentsCoordinates[2],
                                MapSizexy));
                glVertex3f(RealToGlCoord_3D(NeighboursCoordinates[0],
                                MapSizexy),
                        RealToGlCoord_3D(NeighboursCoordinates[1], MapSizexy),
                        RealToGlCoord_3D(NeighboursCoordinates[2], MapSizexy));

                glEnd();

            }

        }
    }

}

/* 3D camera movement */

void TranslateCameraOnXYPlane(vizmode_params_t * VizParams, double *Direction,
        const double StepSize) {

    /* Projection onto XY plane */
    Direction[2] = 0.0;
    UnitVect(Direction, Direction);

    /* Translation */
    VizParams->CenterX += Direction[0] * StepSize;
    VizParams->CenterY += Direction[1] * StepSize;
    VizParams->EyeX += Direction[0] * StepSize;
    VizParams->EyeY += Direction[1] * StepSize;

}

void RotateCameraAroundCenter(vizmode_params_t * VizParams, double *Axis,
        const double angle) {

    /* Creating eye from center vector */
    FillVect(EyeFromCenter,
            VizParams->EyeX - VizParams->CenterX,
            VizParams->EyeY - VizParams->CenterY,
            VizParams->EyeZ - VizParams->CenterZ);

    /* Rotation */
    RotateVectAroundSpecificAxis(EyeFromCenter, EyeFromCenter, Axis, angle);
    NewEyeZ = VizParams->CenterZ + EyeFromCenter[2];

    DistanceFromCenter = VectAbsXY(EyeFromCenter);

    if (VizParams->DistanceFromCenterLimit_XY > DistanceFromCenter ||
            VizParams->EyeZLimit > NewEyeZ) {
        return;
    } else {
        VizParams->EyeX = VizParams->CenterX + EyeFromCenter[0];
        VizParams->EyeY = VizParams->CenterY + EyeFromCenter[1];
        VizParams->EyeZ = NewEyeZ;
    }

}

void ZoomOnCenter(vizmode_params_t * VizParams, const double StepSize) {

    /* Negative StepSize means zoom in... */

    FillVect(EyeFromCenter, VizParams->EyeX - VizParams->CenterX,
            VizParams->EyeY - VizParams->CenterY,
            VizParams->EyeZ - VizParams->CenterZ);
    DistanceFromCenter = VectAbs(EyeFromCenter);
    UnitVect(EyeFromCenter, EyeFromCenter);
    NewEyeZ = EyeFromCenter[2] * StepSize + VizParams->EyeZ;

    if (DistanceFromCenter + StepSize < VizParams->DistanceFromCenterLimit
            || NewEyeZ < VizParams->EyeZLimit) {
        return;
    } else {
        VizParams->EyeX += EyeFromCenter[0] * StepSize;
        VizParams->EyeY += EyeFromCenter[1] * StepSize;
        VizParams->EyeZ = NewEyeZ;
    }

}
