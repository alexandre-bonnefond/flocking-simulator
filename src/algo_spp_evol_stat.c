//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/* SPP model statistic functions
 */

#include "algo_stat.h"
#include "algo_spp_evol.h"

/* Macro for saving standard deviations at the end of the simulation */
// Place it inside the CloseModelSpecificStats function!
static double StDev_Temp;
#define SAVE_STDEV(stat, file) \
    StDev_Temp = Data_##stat##_StDev / ElapsedTime - pow(Data_##stat##_Sum / ElapsedTime, 2); \
    if (StDev_Temp < 0.0) { \
        StDev_Temp = 0.0; \
    } \
    fprintf (f_##file##_StDev, "\t%lf", sqrt(StDev_Temp));
#define SAVE_DISTANCEFROMARENA_STDEV(stat, file) \
    StDev_Temp = Data_##stat##_StDev / TimeElapsedNearArena- pow(Data_##stat##_Sum / TimeElapsedNearArena, 2); \
    if (StDev_Temp < 0.0) { \
        StDev_Temp = 0.0; \
    } \
    fprintf (f_##file##_StDev, "\t%lf", sqrt(StDev_Temp));

FILE *f_DistanceFromArenaFile;
FILE *f_ClusterDependentParams;
FILE *f_ClusterParams;
FILE *f_ClusterRP;

FILE *f_DistanceFromArenaFile_StDev;
FILE *f_ClusterDependentParams_StDev;
FILE *f_ClusterParams_StDev;
FILE *f_ClusterRP_StDev;

double **Adjacency;
static int Dimension;
bool *Visited;                  // for DFS algorithm

double TimeElapsedNearArena = 0.0;

/* Variables for storing time averages */
double Data_Corr_Sum = 0.0, Data_CorrStd_Sum = 0.0, Data_CorrMin_Sum =
        0.0, Data_CorrMax_Sum = 0.0;
double Data_RP_Sum = 0.0, Data_RPStd_Sum = 0.0, Data_RPMin_Sum =
        0.0, Data_RPMax_Sum = 0.0;
double Data_MinCluster_Sum = 0.0;
double Data_MaxCluster_Sum = 0.0;
double Data_IndependentAgents_Sum = 0;
double Data_DistanceFromArena_Sum = 0.0, Data_DistanceFromArenaMin_Sum = 0.0,
        Data_DistanceFromArenaMax_Sum = 0.0, Data_DistanceFromArenaStd_Sum =
        0.0;

/* Variables for storing standard deviations */
double Data_Corr_StDev = 0.0, Data_CorrStd_StDev = 0.0, Data_CorrMin_StDev =
        0.0, Data_CorrMax_StDev = 0.0;
double Data_RP_StDev = 0.0, Data_RPStd_StDev = 0.0, Data_RPMin_StDev =
        0.0, Data_RPMax_StDev = 0.0;
double Data_MinCluster_StDev = 0.0, Data_MaxCluster_StDev =
        0.0, Data_IndependentAgents_StDev = 0.0;
double Data_DistanceFromArena_StDev = 0.0, Data_DistanceFromArenaMin_StDev =
        0.0, Data_DistanceFromArenaMax_StDev =
        0.0, Data_DistanceFromArenaStd_StDev = 0.0;

double n_Avg = 0.0, n_StDev = 0.0;

/* Function for opening stat files, creating header lines, etc. */
void InitializeModelSpecificStats(stat_utils_t * StatUtils) {

    /* Initializing output files */
    INITIALIZE_OUTPUT_FILE("distance_from_arena.dat", f_DistanceFromArenaFile);
    INITIALIZE_OUTPUT_FILE("cluster_dependent_correlation.dat",
            f_ClusterDependentParams);
    INITIALIZE_OUTPUT_FILE("cluster_parameters.dat", f_ClusterParams);
    INITIALIZE_OUTPUT_FILE("cluster_dependent_received_power.dat", f_ClusterRP);

    /* Headers */
    if (STAT != StatUtils->SaveMode && STEADYSTAT != StatUtils->SaveMode) {
        fprintf(f_DistanceFromArenaFile,
                "time_(s)\tdistance_from_arena_avg_(cm)\tdistance_from_arena_stdev_(cm)\tdistance_from_arena_min_(cm)\tdistance_from_arena_max_(cm)\tnumber_of_agents_outside\n");
    }
    fprintf(f_ClusterDependentParams,
            "time_(s)\tcluster_dependent_correlation_avg\tcluster_dependent_correlation_stdev\tcluster_dependent_correlation_min\tcluster_dependent_correlation_max\n");
    fprintf(f_ClusterParams,
            "time_(s)\tmin_cluster_size\tmax_cluster_size\tagents_not_in_cluster\tnumber_of_clusters\n");
    fprintf(f_ClusterRP,
            "time_(s)\tcluster_dependent_received_power_avg\tcluster_dependent_received_power_stdev\tcluster_dependent_received_power_min\tcluster_dependent_received_power_max\n");

    if (STAT == StatUtils->SaveMode || STEADYSTAT == StatUtils->SaveMode) {

        fprintf(f_DistanceFromArenaFile,
                "time_elapsed_near_arena_(s)\tdistance_from_arena_avg_(cm)\tdistance_from_arena_stdev_(cm)\tdistance_from_arena_min_(cm)\tdistance_from_arena_max_(cm)\tnumber_of_agents_outside\n");

        INITIALIZE_OUTPUT_FILE("distance_from_arena_stdev.dat",
                f_DistanceFromArenaFile_StDev);
        INITIALIZE_OUTPUT_FILE("cluster_dependent_correlation_stdev.dat",
                f_ClusterDependentParams_StDev);
        INITIALIZE_OUTPUT_FILE("cluster_parameters_stdev.dat",
                f_ClusterParams_StDev);
        INITIALIZE_OUTPUT_FILE("cluster_dependent_received_power_stdev.dat",
                f_ClusterRP_StDev);

        fprintf(f_DistanceFromArenaFile_StDev,
                "This file contains standard deviations. Check out \"distance_from_arena.dat\" for more details!\n");
        fprintf(f_ClusterDependentParams_StDev,
                "This file contains standard deviations. Check out \"cluster_dependent_parameters.dat\" for more details!\n");
        fprintf(f_ClusterParams_StDev,
                "This file contains standard deviations. Check out \"cluster_parameters.dat\" for more details!\n");
        fprintf(f_ClusterRP_StDev,
                "This file contains standard deviations. Check out \"cluster_dependent_received_power.dat\" for more details!\n");

        fprintf(f_DistanceFromArenaFile_StDev,
                "time_elapsed_near_arena_(s)\tdistance_from_arena_avg_(cm)\tdistance_from_arena_stdev_(cm)\tdistance_from_arena_min_(cm)\tdistance_from_arena_max_(cm)\tnumber_of_agents_outside\n");
        fprintf(f_ClusterDependentParams_StDev,
                "time_(s)\tcluster_dependent_correlation_avg\tcluster_dependent_correlation_stdev\tcluster_dependent_correlation_min\tcluster_dependent_correlation_max\n");
        fprintf(f_ClusterParams_StDev,
                "time_(s)\tmin_cluster_size\tmax_cluster_size\tagents_not_in_cluster\n");
        fprintf(f_ClusterRP_StDev,
                "time_(s)\tcluster_dependent_received_power_avg\tcluster_dependent_received_power_stdev\tcluster_dependent_received_power_min\tcluster_dependent_received_power_max\n");
    }

    TimeElapsedNearArena = 0.0;

}

// TODO TODO TODO: Construction of adjacency matrix is not optimal...
// Graph should be constructed other way...
void ConstructAdjacency(double **OutputAdjacency, phase_t * Phase,
        const double CommunicationRange) {

    int i, j;
    double *AgentsCoordinates;
    double *NeighboursCoordinates;
    static double Difference[3];

    for (i = 0; i < Phase->NumberOfAgents; i++) {

        AgentsCoordinates = Phase->Coordinates[i];

        /* The adjacency matrix is symmetric */
        for (j = 0; j < i; j++) {

            NeighboursCoordinates = Phase->Coordinates[j];
            VectDifference(Difference, AgentsCoordinates,
                    NeighboursCoordinates);

            if (VectAbs(Difference) < CommunicationRange) {
                OutputAdjacency[i][j] = OutputAdjacency[j][i] = 1;
            } else {
                OutputAdjacency[i][j] = OutputAdjacency[j][i] = 0;
            }

        }

    }

}

void CreateCluster(const int i, double **InputAdjacency,
        const int NumberOfAgents, unit_model_params_t * UnitParams) {

    int k;

    Visited[i] = true;

    for (k = 0; k < NumberOfAgents; k++) {
        if (UnitParams->communication_type.Value == 1 || UnitParams->communication_type.Value == 2) {
            if (InputAdjacency[i][k] >= UnitParams->sensitivity_thresh.Value || InputAdjacency[k][i] >= UnitParams->sensitivity_thresh.Value) {
                if (Visited[k] != true) {
                    CreateCluster(k, InputAdjacency, NumberOfAgents, UnitParams);
                }
            }
        }
        else if (UnitParams->communication_type.Value == 0) {
            if (InputAdjacency[i][k] == 1) {
                if (Visited[k] != true) {
                    CreateCluster(k, InputAdjacency, NumberOfAgents, UnitParams);
                }
            }
        }
    }
}

// helper function to return interaction range
double GetInteractionRange() {
    // old version
    // return R_0 + R_0_Offset_Frict;
    // linsqrt version, with max dv = 2*V_Flock
    // return R_0 + R_0_Offset_Frict + StoppingDistanceLinSqrt(V_Flock*2, Acc_Frict, Slope_Frict);
    // linsqrt version, with max dv = V_Flock
    return R_0 + R_0_Offset_Frict + StoppingDistanceLinSqrt(V_Flock, Acc_Frict,
            Slope_Frict);
    // linear part of linsqrt version
    // return R_0 + R_0_Offset_Frict + Acc_frict/Slope_Frict;
}

void SaveClusterDependentParams(phase_t * Phase, sit_parameters_t * SitParams,
        unit_model_params_t * UnitParams, stat_utils_t * StatUtils) {


    Dimension = SitParams->NumberOfAgents;
    Visited = BooleanData(Dimension);

    int i, j;
    static double AgentsCoordinates[3];
    static double NeighboursCoordinates[3];
    static double Difference[3];

    static int NumberOfAgentsInCluster;
    static int NumberOfAgentsInithCluster;
    static int AgentsNotInCluster;
    static int MaxClusterSize;
    static int MinClusterSize;
    static int NumberOfCluster;

    static double Avg_Corr, Avg_RP;
    static double StDev_Corr, StDev_RP;
    static double Min_Corr, Min_RP, Max_Corr, Max_RP;
    static int MaxSize;
    Min_Corr = Min_RP = 2e222;
    Max_Corr = 0.0;
    Max_RP = -2e222;
    static double Temp_Corr, Temp_RP;
    Avg_Corr = Avg_RP = 0.0;
    StDev_Corr = StDev_RP = 0.0;
    
    static bool IsItInCluster;
    NumberOfAgentsInCluster = 0;
    AgentsNotInCluster = 0;
    MaxClusterSize = 0;
    MinClusterSize = 0;
    NumberOfCluster = 0;

    if (UnitParams->communication_type.Value == 0) {
        Adjacency = doubleMatrix(Dimension, Dimension);
        ConstructAdjacency(Adjacency, Phase, UnitParams->R_C.Value);
    }
    

    int Labels[SitParams->NumberOfAgents];
    for (i = 0; i < SitParams->NumberOfAgents; i++) {
        Labels[i] = i;
    }    

    /* Calculating correlations only inside clusters */
    for (i = 0; i < SitParams->NumberOfAgents; i++) {

        NumberOfAgentsInithCluster = 1;
        //NumberOfAgentsInCluster += 1;
        IsItInCluster = false;

        int incr = 0;
        int k;
        for (k = 0; k < SitParams->NumberOfAgents; k++) {
            Visited[k] = false;
        }

        if (UnitParams->communication_type.Value == 1 || UnitParams->communication_type.Value == 2) {
            CreateCluster(i, Phase->Laplacian, Phase->NumberOfAgents, UnitParams);
            for (k = 0; k < SitParams->NumberOfAgents; k++) {
                if (true == Visited[k]) {
                    NumberOfAgentsInithCluster++;
                }
            }
        }

        else if (UnitParams->communication_type.Value == 0) {
            CreateCluster(i, Adjacency, Phase->NumberOfAgents, UnitParams);
            for (k = 0; k < SitParams->NumberOfAgents; k++) {
                if (true == Visited[k]) {
                    NumberOfAgentsInithCluster++;
                }
            }
        }

        int VisitedIndex[NumberOfAgentsInithCluster - 1];
        for (j = 0; j < SitParams->NumberOfAgents; j++) {
                if (true == Visited[j]) {
                    VisitedIndex[incr] = j;
                    incr++;
                }
        }

        if (InnerSum(Labels, SitParams->NumberOfAgents) != - SitParams->NumberOfAgents) {

            bool AddCluster = false;

            for (j = 0; j < NumberOfAgentsInithCluster - 1; j++) {
                if (Labels[VisitedIndex[j]] != -1) {
                    Labels[VisitedIndex[j]] = -1;
                    AddCluster = true;
                }

            }
            NumberOfCluster += (AddCluster == true ? 1 : 0);

        }
        

        if (NumberOfAgentsInithCluster > 2) {
            IsItInCluster = true;
        }

        GetAgentsVelocity(AgentsCoordinates, Phase, i);
        UnitVect(AgentsCoordinates, AgentsCoordinates);

        for (j = 0; j < i; j++) {

            if (i != j && Visited[j] == true) {
                GetAgentsVelocity(NeighboursCoordinates, Phase, j);
                UnitVect(NeighboursCoordinates, NeighboursCoordinates);
                Temp_Corr = ScalarProduct(AgentsCoordinates, NeighboursCoordinates,
                        3);
                Temp_RP = Phase->Laplacian[i][j];

                Avg_Corr += Temp_Corr;
                Avg_RP += Temp_RP;
                StDev_Corr += Temp_Corr * Temp_Corr;
                StDev_RP += Temp_RP * Temp_RP;

                if (Temp_Corr > Max_Corr) {
                    Max_Corr = Temp_Corr;
                }
                if (Temp_RP > Max_RP) {
                    Max_RP = Temp_RP;
                }
                if (Temp_Corr < Min_Corr) {
                    Min_Corr = Temp_Corr;
                }
                if (Temp_RP < Min_RP) {
                    Min_RP = Temp_RP;
                }
                NumberOfAgentsInCluster += 1;
            }

        }

        if (NumberOfAgentsInithCluster > MaxClusterSize
                && NumberOfAgentsInithCluster > 1) {
            MaxClusterSize = NumberOfAgentsInithCluster - 1;
        }
        if ((NumberOfAgentsInithCluster < MinClusterSize
                        && NumberOfAgentsInithCluster > 1 && MinClusterSize > 0)
                || (MinClusterSize == 0 && NumberOfAgentsInithCluster > 1)) {
            MinClusterSize = NumberOfAgentsInithCluster - 1;
        }

        AgentsNotInCluster += (IsItInCluster == true ? 0 : 1);

    }

    if (NumberOfAgentsInCluster > 0) {
        Avg_Corr /= NumberOfAgentsInCluster;
        StDev_Corr /= NumberOfAgentsInCluster;
        StDev_Corr -= Avg_Corr * Avg_Corr;

        Avg_RP /= NumberOfAgentsInCluster;
        StDev_RP /= NumberOfAgentsInCluster;
        StDev_RP -= Avg_RP * Avg_RP;
    } else {
        Avg_Corr = 0.0;
        StDev_Corr = 0.0;
        Min_Corr = 0.0;
        Max_Corr = 0.0;

        Avg_RP = 0.0;
        StDev_RP = 0.0;
        Min_RP = 0.0;
        Max_RP = 0.0;
    }

    if (StDev_Corr < 0.0) {
        StDev_Corr = 0.0;
    }

    if (TIMELINE == StatUtils->SaveMode) {
        // printf("%d", NumberOfCluster);
        fprintf(f_ClusterDependentParams, "%lf\t%lf\t%lf\t%lf\t%lf\n",
                StatUtils->ElapsedTime, Avg_Corr, sqrt(StDev_Corr), Min_Corr, Max_Corr);
        fprintf(f_ClusterParams, "%lf\t%d\t%d\t%d\t%d\n",
                StatUtils->ElapsedTime, MinClusterSize, MaxClusterSize,
                AgentsNotInCluster, NumberOfCluster);
        fprintf(f_ClusterRP, "%lf\t%lf\t%lf\t%lf\t%lf\n",
                StatUtils->ElapsedTime, Avg_RP, sqrt(StDev_RP), Min_RP, Max_RP);

    } else if (StatUtils->SaveMode != STEADYSTAT
            || StatUtils->ElapsedTime > SitParams->StartOfSteadyState) {

        Data_Corr_Sum += Avg_Corr * SitParams->DeltaT;
        Data_CorrStd_Sum += sqrt(StDev_Corr) * SitParams->DeltaT;
        Data_CorrMin_Sum += Min_Corr * SitParams->DeltaT;
        Data_CorrMax_Sum += Max_Corr * SitParams->DeltaT;

        Data_MinCluster_Sum += MinClusterSize * SitParams->DeltaT;
        Data_MaxCluster_Sum += MaxClusterSize * SitParams->DeltaT;
        Data_IndependentAgents_Sum += AgentsNotInCluster * SitParams->DeltaT;

        Data_Corr_StDev += Avg_Corr * Avg_Corr * SitParams->DeltaT;
        Data_CorrStd_StDev += StDev_Corr * SitParams->DeltaT;
        Data_CorrMin_StDev += Min_Corr * Min_Corr * SitParams->DeltaT;
        Data_CorrMax_StDev += Max_Corr * Max_Corr * SitParams->DeltaT;

        Data_MinCluster_StDev +=
                MinClusterSize * MinClusterSize * SitParams->DeltaT;
        Data_MaxCluster_StDev +=
                MaxClusterSize * MaxClusterSize * SitParams->DeltaT;
        Data_IndependentAgents_StDev +=
                AgentsNotInCluster * AgentsNotInCluster * SitParams->DeltaT;

        Data_RP_Sum += Avg_RP * SitParams->DeltaT;
        Data_RPStd_Sum += sqrt(StDev_RP) * SitParams->DeltaT;
        Data_RPMin_Sum += Min_RP * SitParams->DeltaT;
        Data_RPMax_Sum += Max_RP * SitParams->DeltaT;

        Data_RP_StDev += Avg_RP * Avg_RP * SitParams->DeltaT;
        Data_RPStd_StDev += StDev_RP * SitParams->DeltaT;
        Data_RPMin_StDev += Min_RP * Min_RP * SitParams->DeltaT;
        Data_RPMax_StDev += Max_RP * Max_RP * SitParams->DeltaT;

    }

}

void SaveModelSpecificStats(phase_t * Phase,
        stat_utils_t * StatUtils, unit_model_params_t * UnitParams,
        flocking_model_params_t * FlockingParams,
        sit_parameters_t * SitParams) {

    /* Calculating distance from Arena */
    int i;
    static int n;
    n = 0;

    static double avg, stdev, min, max;
    min = 2e222;
    max = 0.0;
    avg = 0.0;
    stdev = 0.0;

    double *AgentsCoordinates;
    static double ArenaCoordinates[3];
    static double TempCoords[3];

    FillVect(ArenaCoordinates, ArenaCenterX, ArenaCenterY, 0.0);

    for (i = 0; i < SitParams->NumberOfAgents; i++) {

        static double dist;

        //GetAgentsCoordinates (AgentsCoordinates, Phase, i);
        AgentsCoordinates = Phase->Coordinates[i];

        /* Distance from arena depends on the shape of the arena */
        if (0.0 == ArenaShape) {        // Sphere-shaped arena

            VectDifference(TempCoords, ArenaCoordinates, AgentsCoordinates);
            dist = VectAbs(TempCoords);
            dist = (dist > ArenaRadius ? dist - ArenaRadius : 0.0);
        } else if (1.0 == ArenaShape) { // Cube-shaped arena

            static double FromSide[3];

            VectDifference(TempCoords, ArenaCoordinates, AgentsCoordinates);

            int j;
            for (j = 0; j < 3; j++) {
                FromSide[j] = (TempCoords[j] < ArenaRadius
                        && TempCoords[j] >
                        -ArenaRadius ? 0.0 : fabs(TempCoords[j]) - ArenaRadius);
            }

            dist = VectAbs(FromSide);

        }

        if (dist > 0.0) {
            if (dist > max) {
                max = dist;
            }
            if (dist < min) {
                min = dist;
            }
            avg += dist;
            stdev += pow(dist, 2);
            n++;
        }

    }

    if (n != 0.0) {
        avg /= n;
        stdev /= n;

        stdev -= avg * avg;
    } else {
        avg = 0.0;
        stdev = 0.0;
        max = 0.0;
        min = 0.0;
    }

    if (stdev < 0.0) {
        stdev = 0.0;
    }

    if (TIMELINE == StatUtils->SaveMode) {
        if (avg > 0.0 || 0.0 == StatUtils->ElapsedTime) {
            fprintf(f_DistanceFromArenaFile, "%lf\t%lf\t%lf\t%lf\t%lf\t%d\n",
                    StatUtils->ElapsedTime, avg, sqrt(stdev), min, max, n);
        }
    } else if (StatUtils->SaveMode != STEADYSTAT
            || StatUtils->ElapsedTime > SitParams->StartOfSteadyState) {
        if (avg > 0.0 || 0.0 == n_Avg) {
            Data_DistanceFromArena_Sum += avg * SitParams->DeltaT;
            Data_DistanceFromArenaStd_Sum += sqrt(stdev) * SitParams->DeltaT;
            Data_DistanceFromArenaMin_Sum += min * SitParams->DeltaT;
            Data_DistanceFromArenaMax_Sum += max * SitParams->DeltaT;

            Data_DistanceFromArena_StDev += avg * avg * SitParams->DeltaT;
            Data_DistanceFromArenaStd_StDev += stdev * SitParams->DeltaT;
            Data_DistanceFromArenaMin_StDev += min * min * SitParams->DeltaT;
            Data_DistanceFromArenaMax_StDev += max * max * SitParams->DeltaT;

            n_Avg += n * SitParams->DeltaT;
            n_StDev += n * n * SitParams->DeltaT;

            TimeElapsedNearArena += SitParams->DeltaT;
        }
    }

    SaveClusterDependentParams(Phase, SitParams, UnitParams, StatUtils);

}

/* For closing stat files */
void CloseModelSpecificStats(stat_utils_t * StatUtils, unit_model_params_t * UnitParams) {

    if (STAT == StatUtils->SaveMode || STEADYSTAT == StatUtils->SaveMode) {
        double ElapsedTime = StatUtils->ElapsedTime;
        if (STEADYSTAT == StatUtils->SaveMode) {
            ElapsedTime -= StatUtils->StartOfSteadyState;
        }
        /* Averages */
        fprintf(f_ClusterDependentParams, "%lf\t%lf\t%lf\t%lf\t%lf\n",
                ElapsedTime, Data_Corr_Sum / ElapsedTime,
                Data_CorrStd_Sum / ElapsedTime, Data_CorrMin_Sum / ElapsedTime,
                Data_CorrMax_Sum / ElapsedTime);
        printf("\t\t\t\t\t\t%f\n", Data_MaxCluster_Sum);
        fprintf(f_ClusterParams, "%lf\t%lf\t%lf\t%lf\n", ElapsedTime,
                Data_MinCluster_Sum / ElapsedTime,
                Data_MaxCluster_Sum / ElapsedTime,
                Data_IndependentAgents_Sum / ElapsedTime);
        fprintf(f_ClusterRP, "%lf\t%lf\t%lf\t%lf\t%lf\n",
                ElapsedTime, Data_RP_Sum / ElapsedTime,
                Data_RPStd_Sum / ElapsedTime, Data_RPMin_Sum / ElapsedTime,
                Data_RPMax_Sum / ElapsedTime);
        fprintf(f_DistanceFromArenaFile, "%lf\t%lf\t%lf\t%lf\t%lf\t",
                TimeElapsedNearArena,
                Data_DistanceFromArena_Sum / TimeElapsedNearArena,
                Data_DistanceFromArenaStd_Sum / TimeElapsedNearArena,
                Data_DistanceFromArenaMin_Sum / TimeElapsedNearArena,
                Data_DistanceFromArenaMax_Sum / TimeElapsedNearArena);
        fprintf(f_DistanceFromArenaFile, "%lf\n", n_Avg / TimeElapsedNearArena);

        /* Standard deviations */

        fprintf(f_ClusterDependentParams_StDev, "%lf", ElapsedTime);
        SAVE_STDEV(Corr, ClusterDependentParams);
        SAVE_STDEV(CorrStd, ClusterDependentParams);
        SAVE_STDEV(CorrMin, ClusterDependentParams);
        SAVE_STDEV(CorrMax, ClusterDependentParams);
        fprintf(f_ClusterDependentParams_StDev, "\n");

        fprintf(f_ClusterRP_StDev, "%lf", ElapsedTime);
        SAVE_STDEV(RP, ClusterRP);
        SAVE_STDEV(RPStd, ClusterRP);
        SAVE_STDEV(RPMin, ClusterRP);
        SAVE_STDEV(RPMax, ClusterRP);
        fprintf(f_ClusterRP_StDev, "\n");

        fprintf(f_ClusterParams_StDev, "%lf", ElapsedTime);
        SAVE_STDEV(MinCluster, ClusterParams);
        SAVE_STDEV(MaxCluster, ClusterParams);
        SAVE_STDEV(IndependentAgents, ClusterParams);
        fprintf(f_ClusterParams_StDev, "\n");

        fprintf(f_DistanceFromArenaFile_StDev, "%lf", TimeElapsedNearArena);
        SAVE_DISTANCEFROMARENA_STDEV(DistanceFromArena, DistanceFromArenaFile);
        SAVE_DISTANCEFROMARENA_STDEV(DistanceFromArenaStd,
                DistanceFromArenaFile);
        SAVE_DISTANCEFROMARENA_STDEV(DistanceFromArenaMin,
                DistanceFromArenaFile);
        SAVE_DISTANCEFROMARENA_STDEV(DistanceFromArenaMax,
                DistanceFromArenaFile);

        StDev_Temp =
                (n_StDev / TimeElapsedNearArena) -
                pow(n_Avg / TimeElapsedNearArena, 2);
        if (StDev_Temp < 0.0) {
            StDev_Temp = 0.0;
        }
        fprintf(f_DistanceFromArenaFile_StDev, "\t%lf\n", sqrt(StDev_Temp));

        fclose(f_DistanceFromArenaFile_StDev);
        fclose(f_ClusterDependentParams_StDev);
        fclose(f_ClusterParams_StDev);
        fclose(f_ClusterRP_StDev);

    }

    fclose(f_DistanceFromArenaFile);
    fclose(f_ClusterDependentParams);
    fclose(f_ClusterParams);
    fclose(f_ClusterRP);

    if (UnitParams->communication_type.Value == 0) {
        freeMatrix(Adjacency, Dimension, Dimension);
    }
    free(Visited);

}
