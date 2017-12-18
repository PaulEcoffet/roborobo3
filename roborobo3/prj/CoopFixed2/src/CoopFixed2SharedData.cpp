/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <limits>
#include <core/RoboroboMain/main.h>
#include "CoopFixed2/include/CoopFixed2SharedData.h"


int CoopFixed2SharedData::evaluationTime = 1000;
int CoopFixed2SharedData::genomeLog = 2000;
int CoopFixed2SharedData::controllerType = 0;
double CoopFixed2SharedData::maxTranslationalValue = 2;
double CoopFixed2SharedData::maxRotationalVelocity = 30;
int CoopFixed2SharedData::nbHiddenLayers = 1;
int CoopFixed2SharedData::nbNeuronsPerHiddenLayer = 10;
int CoopFixed2SharedData::nbEvaluationsPerGeneration = 1;
int CoopFixed2SharedData::takeVideoEveryGeneration = std::numeric_limits<int>::max();
int CoopFixed2SharedData::seeCoopFromDist = 0;
bool CoopFixed2SharedData::prisonerDilemma = false;

void CoopFixed2SharedData::initSharedData()
{
    gProperties.checkAndGetPropertyValue("evaluationTime", &CoopFixed2SharedData::evaluationTime, true);
    gProperties.checkAndGetPropertyValue("nbEvaluationsPerGeneration", &CoopFixed2SharedData::nbEvaluationsPerGeneration, true);
    gProperties.checkAndGetPropertyValue("genomeLog", &CoopFixed2SharedData::genomeLog, true);
    gProperties.checkAndGetPropertyValue("controllerType", &CoopFixed2SharedData::controllerType, true);
    gProperties.checkAndGetPropertyValue("gMaxTranslationalSpeed", &CoopFixed2SharedData::maxTranslationalValue, true);
    gProperties.checkAndGetPropertyValue("gMaxRotationalSpeed", &CoopFixed2SharedData::maxRotationalVelocity, true);
    gProperties.checkAndGetPropertyValue("nbHiddenLayers", &CoopFixed2SharedData::nbHiddenLayers, true);
    gProperties.checkAndGetPropertyValue("nbNeuronsPerHiddenLayer", &CoopFixed2SharedData::nbNeuronsPerHiddenLayer, true);
    gProperties.checkAndGetPropertyValue("takeVideoEveryGeneration", &CoopFixed2SharedData::takeVideoEveryGeneration, false);
    gProperties.checkAndGetPropertyValue("seeCoopFromDist", &CoopFixed2SharedData::seeCoopFromDist, false);
    gProperties.checkAndGetPropertyValue("prisonerDilemma", &CoopFixed2SharedData::prisonerDilemma, false);

}


