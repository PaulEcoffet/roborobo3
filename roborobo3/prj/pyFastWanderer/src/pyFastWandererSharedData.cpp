//
// Created by paul on 31/10/17.
//

#include <limits>
#include <RoboroboMain/main.h>
#include "pyFastWanderer/include/pyFastWandererSharedData.h"


int pyFastWandererSharedData::evaluationTime = 2000;
int pyFastWandererSharedData::controllerType = 2;
double pyFastWandererSharedData::maxTranslationalValue = 2;
double pyFastWandererSharedData::maxRotationalVelocity = 2;
int pyFastWandererSharedData::nbHiddenLayers = 1;
int pyFastWandererSharedData::nbNeuronsPerHiddenLayer = 10;

void pyFastWandererSharedData::initSharedData()
{
    gProperties.checkAndGetPropertyValue("evaluationTime", &pyFastWandererSharedData::evaluationTime, true);
    gProperties.checkAndGetPropertyValue("controllerType", &pyFastWandererSharedData::controllerType, true);
    gProperties.checkAndGetPropertyValue("gMaxTranslationalSpeed", &pyFastWandererSharedData::maxTranslationalValue, true);
    gProperties.checkAndGetPropertyValue("gMaxRotationalSpeed", &pyFastWandererSharedData::maxRotationalVelocity, true);
    gProperties.checkAndGetPropertyValue("nbHiddenLayers", &pyFastWandererSharedData::nbHiddenLayers, true);
    gProperties.checkAndGetPropertyValue("nbNeuronsPerHiddenLayer", &pyFastWandererSharedData::nbNeuronsPerHiddenLayer, true);
}


