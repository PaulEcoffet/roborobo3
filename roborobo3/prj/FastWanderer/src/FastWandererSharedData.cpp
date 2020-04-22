//
// Created by paul on 31/10/17.
//

#include <limits>
#include <RoboroboMain/main.h>
#include "FastWanderer/include/FastWandererSharedData.h"


int FastWandererSharedData::evaluationTime = 2000;
int FastWandererSharedData::controllerType = 2;
double FastWandererSharedData::maxTranslationalValue = 2;
double FastWandererSharedData::maxRotationalVelocity = 2;
int FastWandererSharedData::nbHiddenLayers = 1;
int FastWandererSharedData::nbNeuronsPerHiddenLayer = 10;

void FastWandererSharedData::initSharedData()
{
    gProperties.checkAndGetPropertyValue("evaluationTime", &FastWandererSharedData::evaluationTime, true);
    gProperties.checkAndGetPropertyValue("controllerType", &FastWandererSharedData::controllerType, true);
    gProperties.checkAndGetPropertyValue("gMaxTranslationalSpeed", &FastWandererSharedData::maxTranslationalValue,
                                         true);
    gProperties.checkAndGetPropertyValue("gMaxRotationalSpeed", &FastWandererSharedData::maxRotationalVelocity, true);
    gProperties.checkAndGetPropertyValue("nbHiddenLayers", &FastWandererSharedData::nbHiddenLayers, true);
    gProperties.checkAndGetPropertyValue("nbNeuronsPerHiddenLayer", &FastWandererSharedData::nbNeuronsPerHiddenLayer,
                                         true);
}


