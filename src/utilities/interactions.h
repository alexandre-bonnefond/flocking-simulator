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

/* attraction */

void AttractionLin(double *OutputVelocity,
        phase_t * Phase, const double V_Rep_l, const double p_l,
        const double R_0_l, const int WhichAgent, const int Dim_l,
        const bool normalize);

/* Gradient based term from Olfati Saber theory */
void GradientBased(double *OutputVelocity,
        phase_t * Phase, const double epsilon, const double a,
        const double b, const double h,
        const double d, const double r,
        const int WhichAgent, const int Dim_l);

#endif
