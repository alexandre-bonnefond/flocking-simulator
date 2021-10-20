//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/*
 * This file contains unversal interaction terms.
 */

#ifndef INTERACTIONS_H
#define INTERACTIONS_H

#include "math_utils.h"
#include "dynamics_utils.h"
#include "arenas.h"

/* friction / alignment */

void FrictionLinSqrt(double *OutputVelocity, phase_t * Phase,
        const double C_Frict_l, const double V_Frict_l,
        const double Acc_l, const double p_l,
        const double R_0_l, const int WhichAgent, const int Dim_l);

/* repulsion */

void RepulsionLin(double *OutputVelocity,
        phase_t * Phase, const double V_Rep_l, const double p_l,
        const double R_0_l, const int WhichAgent, const int Dim_l,
        const bool normalize);

void RepulsionPowLin(double *OutputVelocity,
        phase_t * Phase, const double ActualTime, const double V_Rep_l, const double p_l,
        const double RP_max, const int WhichAgent, const int Dim_l,
        const bool normalize);

void PressureRepulsion(double *OutputVelocity, 
            phase_t * Phase, const double k, const int WhichAgent, const int Dim_l, const double R_0);
        
/* attraction */

void AttractionLin(double *OutputVelocity,
        phase_t * Phase, const double V_Rep_l, const double p_l,
        const double R_0_l, const int WhichAgent, const int Dim_l,
        const bool normalize);

void AttractionPowLin(double *OutputVelocity,
        phase_t * Phase, const double ActualTime, const double V_Rep_l, const double p_l,
        const double RP_min, const int WhichAgent, const int Dim_l,
        const bool normalize);


/* Smooth pairwise potential with finite cut-off  (see Olfati function) */
double ActionFunction(double z, double a, double b);

/* Gradient based term from Olfati Saber theory */
void GradientBased(double *OutputVelocity,
        phase_t * Phase, const double epsilon, const double a,
        const double b, const double h,
        const double d, const double r,
        const int WhichAgent, const int Dim_l);

void AlignmentOlfati(double *OutputVelocity,
        phase_t * Phase, const double h,
        const double r, const int WhichAgent, 
        const int Dim_l, const double epsilon);

void TrackingOlfati(double *OutputVelocity, double *TargetPosition,
        double *TargetVelocity, phase_t * Phase, const int WhichAgent, 
        const int Dim_l);


/* Target tracking function */
void TargetTracking(double *OutputVelocity, double *TargetPosition,
        phase_t * Phase, const double R_CoM, const double d_CoM,
        const double R_trg, const double d_trg, 
        const int SizeOfNeighbourhood, const int WhichAgent, 
        const int Dim_l);
        
#endif
