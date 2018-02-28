//
// Created by paul on 31/10/17.
//

#include <limits>
#include <RoboroboMain/main.h>
#include "CorrectRepartition/include/CorrectRepartitionSharedData.h"


int CorrectRepartitionSharedData::evaluationTime = 2000;
int CorrectRepartitionSharedData::genomeLog = 2000;
int CorrectRepartitionSharedData::controllerType = 2;
double CorrectRepartitionSharedData::maxTranslationalValue = 2;
double CorrectRepartitionSharedData::maxRotationalVelocity = 2;
int CorrectRepartitionSharedData::nbHiddenLayers = 1;
int CorrectRepartitionSharedData::nbNeuronsPerHiddenLayer = 10;

int CorrectRepartitionSharedData::nbEvaluationsPerGeneration = 1;
int CorrectRepartitionSharedData::takeVideoEveryGeneration = std::numeric_limits<int>::max();

bool CorrectRepartitionSharedData::arrivalMemory = false;
bool CorrectRepartitionSharedData::exactlyTwo = false;
bool CorrectRepartitionSharedData::clones = false;
bool CorrectRepartitionSharedData::ifThreeNoGain = false;


void CorrectRepartitionSharedData::initSharedData()
{
    gProperties.checkAndGetPropertyValue("evaluationTime", &CorrectRepartitionSharedData::evaluationTime, true);
    gProperties.checkAndGetPropertyValue("genomeLog", &CorrectRepartitionSharedData::genomeLog, true);
    gProperties.checkAndGetPropertyValue("controllerType", &CorrectRepartitionSharedData::controllerType, true);
    gProperties.checkAndGetPropertyValue("gMaxTranslationalSpeed", &CorrectRepartitionSharedData::maxTranslationalValue, true);
    gProperties.checkAndGetPropertyValue("gMaxRotationalSpeed", &CorrectRepartitionSharedData::maxRotationalVelocity, true);
    gProperties.checkAndGetPropertyValue("nbHiddenLayers", &CorrectRepartitionSharedData::nbHiddenLayers, true);
    gProperties.checkAndGetPropertyValue("nbNeuronsPerHiddenLayer", &CorrectRepartitionSharedData::nbNeuronsPerHiddenLayer, true);
    gProperties.checkAndGetPropertyValue("nbEvaluationsPerGeneration", &CorrectRepartitionSharedData::nbEvaluationsPerGeneration, true);
    gProperties.checkAndGetPropertyValue("takeVideoEveryGeneration", &CorrectRepartitionSharedData::takeVideoEveryGeneration, false);
    gProperties.checkAndGetPropertyValue("arrivalMemory", &CorrectRepartitionSharedData::arrivalMemory, false);
    gProperties.checkAndGetPropertyValue("exactlyTwo", &CorrectRepartitionSharedData::exactlyTwo, false);
    gProperties.checkAndGetPropertyValue("clones", &CorrectRepartitionSharedData::clones, false);
    gProperties.checkAndGetPropertyValue("ifThreeNoGain", &CorrectRepartitionSharedData::ifThreeNoGain, false);


}


