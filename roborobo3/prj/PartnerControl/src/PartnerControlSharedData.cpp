//
// Created by paul on 31/10/17.
//

#include <limits>
#include <core/RoboroboMain/main.h>
#include "PartnerControl/include/PartnerControlSharedData.h"


int PartnerControlSharedData::evaluationTime = 2000;
int PartnerControlSharedData::genomeLog = 2000;
double PartnerControlSharedData::sigma = 0.1;
int PartnerControlSharedData::controllerType = 2;
double PartnerControlSharedData::maxTranslationalValue = 2;
double PartnerControlSharedData::maxRotationalVelocity = 2;
int PartnerControlSharedData::nbHiddenLayers = 1;
int PartnerControlSharedData::nbNeuronsPerHiddenLayer = 10;
double PartnerControlSharedData::maxCoop = 1;
int PartnerControlSharedData::nbCoopStep = 2;
double PartnerControlSharedData::constantA = 0.5;
double PartnerControlSharedData::constantK = 1.41;
int PartnerControlSharedData::nbEvaluationsPerGeneration = 1;
int PartnerControlSharedData::takeVideoEveryGeneration = std::numeric_limits<int>::max();
int PartnerControlSharedData::seeCoopFromDist = 0;
int PartnerControlSharedData::nbGenerations = 0;
bool PartnerControlSharedData::gaussianPayoff = false;

void PartnerControlSharedData::initSharedData()
{
    gProperties.checkAndGetPropertyValue("evaluationTime", &PartnerControlSharedData::evaluationTime, true);
    gProperties.checkAndGetPropertyValue("nbGenerations", &PartnerControlSharedData::nbGenerations, true);
    gProperties.checkAndGetPropertyValue("genomeLog", &PartnerControlSharedData::genomeLog, true);
    gProperties.checkAndGetPropertyValue("sigma", &PartnerControlSharedData::sigma, true);
    gProperties.checkAndGetPropertyValue("controllerType", &PartnerControlSharedData::controllerType, true);
    gProperties.checkAndGetPropertyValue("gMaxTranslationalSpeed", &PartnerControlSharedData::maxTranslationalValue, true);
    gProperties.checkAndGetPropertyValue("gMaxRotationalSpeed", &PartnerControlSharedData::maxRotationalVelocity, true);
    gProperties.checkAndGetPropertyValue("nbHiddenLayers", &PartnerControlSharedData::nbHiddenLayers, true);
    gProperties.checkAndGetPropertyValue("nbNeuronsPerHiddenLayer", &PartnerControlSharedData::nbNeuronsPerHiddenLayer, true);
    gProperties.checkAndGetPropertyValue("maxCoop", &PartnerControlSharedData::maxCoop, true);
    gProperties.checkAndGetPropertyValue("nbCoopStep", &PartnerControlSharedData::nbCoopStep, true);
    gProperties.checkAndGetPropertyValue("constantA", &PartnerControlSharedData::constantA, true);
    gProperties.checkAndGetPropertyValue("constantK", &PartnerControlSharedData::constantK, true);
    gProperties.checkAndGetPropertyValue("nbEvaluationsPerGeneration", &PartnerControlSharedData::nbEvaluationsPerGeneration, true);
    gProperties.checkAndGetPropertyValue("takeVideoEveryGeneration", &PartnerControlSharedData::takeVideoEveryGeneration, false);
    gProperties.checkAndGetPropertyValue("seeCoopFromDist", &PartnerControlSharedData::seeCoopFromDist, false);

    gProperties.checkAndGetPropertyValue("gaussianPayoff", &PartnerControlSharedData::gaussianPayoff, false);
    std::cout << "gauss:" << PartnerControlSharedData::gaussianPayoff << "\n";
}


