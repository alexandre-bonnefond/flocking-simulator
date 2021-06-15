#ifndef ALGO_SPP_EVOL_STAT_H
#define ALGO_SPP_EVOL_STAT_H

#include "algo_stat.h"
#include "algo_spp_evol.h"
#include "utilities/datastructs.h"

void CloseModelSpecificStats(stat_utils_t * StatUtils, unit_model_params_t * UnitParams);
void SaveModelSpecificStats(phase_t * Phase, stat_utils_t * StatUtils, unit_model_params_t * UnitParams, flocking_model_params_t * FlockingParams, sit_parameters_t * SitParams);
void SaveCBPToFile(measurement_bundle*** CBP, int agentCount, int size, FILE* file);
void SaveClusterDependentParams(phase_t * Phase, sit_parameters_t * SitParams, unit_model_params_t * UnitParams, stat_utils_t * StatUtils);
double GetInteractionRange();
void CreateCluster(const int i, double **InputAdjacency, const int NumberOfAgents, unit_model_params_t * UnitParams);
void ConstructAdjacency(double **OutputAdjacency, phase_t * Phase, const double CommunicationRange);
void InitializeModelSpecificStats(stat_utils_t * StatUtils);

#endif