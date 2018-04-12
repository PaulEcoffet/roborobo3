/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <limits>
#include <RoboroboMain/main.h>
#include "CoopFixed2/include/CoopFixed2SharedData.h"


int CoopFixed2SharedData::evaluationTime = 1000;
int CoopFixed2SharedData::genomeLog = 2000;
int CoopFixed2SharedData::controllerType = 0;
double CoopFixed2SharedData::maxTranslationalValue = 2;
double CoopFixed2SharedData::maxRotationalVelocity = 30;
int CoopFixed2SharedData::nbHiddenLayers = 1;
int CoopFixed2SharedData::nbNeuronsPerHiddenLayer = 10;
int CoopFixed2SharedData::nbEvaluationsPerGeneration = 1;
bool CoopFixed2SharedData::takeVideo = true;
int CoopFixed2SharedData::seeCoopFromDist = 0;
bool CoopFixed2SharedData::prisonerDilemma = false;
bool CoopFixed2SharedData::selfAAsInput = true;
int CoopFixed2SharedData::oppDecay = -1;
bool CoopFixed2SharedData::fixRobotNb = true;
bool CoopFixed2SharedData::teleportRobots = false;
double CoopFixed2SharedData::maxCoop = 10;
double CoopFixed2SharedData::b = 10;
double CoopFixed2SharedData::meanA = 5;
double CoopFixed2SharedData::stdA = 2;
bool CoopFixed2SharedData::tpToNewObj = false;
bool CoopFixed2SharedData::totalInvAsInput = true;
bool CoopFixed2SharedData::arrivalAsInput = false;
bool CoopFixed2SharedData::ownInvAsInput = true;
bool CoopFixed2SharedData::onlyOtherInTotalInv = false;


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
    gProperties.checkAndGetPropertyValue("takeVideo", &CoopFixed2SharedData::takeVideo, false);
    gProperties.checkAndGetPropertyValue("seeCoopFromDist", &CoopFixed2SharedData::seeCoopFromDist, false);
    gProperties.checkAndGetPropertyValue("prisonerDilemma", &CoopFixed2SharedData::prisonerDilemma, false);
    gProperties.checkAndGetPropertyValue("aAsInput", &CoopFixed2SharedData::selfAAsInput, true);
    gProperties.checkAndGetPropertyValue("oppDecay", &CoopFixed2SharedData::oppDecay, true);
    gProperties.checkAndGetPropertyValue("fixRobotNb", &CoopFixed2SharedData::fixRobotNb, true);
    gProperties.checkAndGetPropertyValue("teleportRobots", &CoopFixed2SharedData::teleportRobots, true);
    gProperties.checkAndGetPropertyValue("maxCoop", &CoopFixed2SharedData::maxCoop, true);
    gProperties.checkAndGetPropertyValue("b", &CoopFixed2SharedData::b, true);
    gProperties.checkAndGetPropertyValue("meanA", &CoopFixed2SharedData::meanA, true);
    gProperties.checkAndGetPropertyValue("stdA", &CoopFixed2SharedData::stdA, true);
    gProperties.checkAndGetPropertyValue("tpToNewObj", &CoopFixed2SharedData::tpToNewObj, false);
    gProperties.checkAndGetPropertyValue("totalInvAsInput", &CoopFixed2SharedData::totalInvAsInput, true);
    gProperties.checkAndGetPropertyValue("arrivalAsInput", &CoopFixed2SharedData::arrivalAsInput, true);
    gProperties.checkAndGetPropertyValue("ownInvAsInput", &CoopFixed2SharedData::ownInvAsInput, true);
    gProperties.checkAndGetPropertyValue("onlyOtherInTotalInv", &CoopFixed2SharedData::onlyOtherInTotalInv, true);
}

