//
// Created by paul on 31/10/17.
//

#include <limits>
#include <RoboroboMain/main.h>
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
int PartnerChoiceSharedData::seeCoopFromDist = 0;

void PartnerChoiceSharedData::initSharedData()
{
    gProperties.checkAndGetPropertyValue("evaluationTime", &PartnerChoiceSharedData::evaluationTime, true);
    gProperties.checkAndGetPropertyValue("genomeLog", &PartnerChoiceSharedData::genomeLog, true);
    gProperties.checkAndGetPropertyValue("sigma", &PartnerChoiceSharedData::sigma, true);
    gProperties.checkAndGetPropertyValue("controllerType", &PartnerChoiceSharedData::controllerType, true);
    gProperties.checkAndGetPropertyValue("gMaxTranslationalSpeed", &PartnerChoiceSharedData::maxTranslationalValue, true);
    gProperties.checkAndGetPropertyValue("gMaxRotationalSpeed", &PartnerChoiceSharedData::maxRotationalVelocity, true);
    gProperties.checkAndGetPropertyValue("nbHiddenLayers", &PartnerChoiceSharedData::nbHiddenLayers, true);
    gProperties.checkAndGetPropertyValue("nbNeuronsPerHiddenLayer", &PartnerChoiceSharedData::nbNeuronsPerHiddenLayer, true);
    gProperties.checkAndGetPropertyValue("maxCoop", &PartnerChoiceSharedData::maxCoop, true);
    gProperties.checkAndGetPropertyValue("nbCoopStep", &PartnerChoiceSharedData::nbCoopStep, true);
    gProperties.checkAndGetPropertyValue("constantA", &PartnerChoiceSharedData::constantA, true);
    gProperties.checkAndGetPropertyValue("constantK", &PartnerChoiceSharedData::constantK, true);
    gProperties.checkAndGetPropertyValue("nbEvaluationsPerGeneration", &PartnerChoiceSharedData::nbEvaluationsPerGeneration, true);
    gProperties.checkAndGetPropertyValue("takeVideoEveryGeneration", &PartnerChoiceSharedData::takeVideoEveryGeneration, false);
    gProperties.checkAndGetPropertyValue("seeCoopFromDist", &PartnerChoiceSharedData::seeCoopFromDist, false);
}


