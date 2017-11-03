//
// Created by paul on 31/10/17.
//

#include <limits>
#include "PartnerChoice/include/PartnerChoiceSharedData.h"


int PartnerChoiceSharedData::evaluationTime = 2000;
int PartnerChoiceSharedData::genomeLog = 2000;
double PartnerChoiceSharedData::sigma = 0.1;
int PartnerChoiceSharedData::controllerType = 2;
double PartnerChoiceSharedData::maxTranslationalValue = 2;
double PartnerChoiceSharedData::maxRotationalVelocity = 2;
int PartnerChoiceSharedData::nbHiddenLayers = 1;
int PartnerChoiceSharedData::nbNeuronsPerHiddenLayer = 10;
double PartnerChoiceSharedData::maxCoop = 1;
int PartnerChoiceSharedData::nbCoopStep = 2;
double PartnerChoiceSharedData::constantA = 0.5;
double PartnerChoiceSharedData::constantK = 1.41;
int PartnerChoiceSharedData::nbEvaluationsPerGeneration = 1;
int PartnerChoiceSharedData::takeVideoEveryGeneration = std::numeric_limits<int>::max();