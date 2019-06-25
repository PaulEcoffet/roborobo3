/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <limits>
#include <RoboroboMain/main.h>
#include "Lion/include/LionSharedData.h"

#define GETVAL(name)  (gProperties.checkAndGetPropertyValue(#name, &LionSharedData::name, false))


int LionSharedData::evaluationTime = 1000;
int LionSharedData::genomeLog = 2000;
int LionSharedData::controllerType = 0;
double LionSharedData::maxTranslationalValue = 2;
double LionSharedData::maxRotationalVelocity = 30;
int LionSharedData::nbHiddenLayers = 1;
int LionSharedData::nbNeuronsPerHiddenLayer = 10;
int LionSharedData::nbEvaluationsPerGeneration = 1;
bool LionSharedData::takeVideo = true;
int LionSharedData::oppDecay = -1;
double LionSharedData::maxCoop = 10;
double LionSharedData::b = 10;
double LionSharedData::meanA = 5;
double LionSharedData::stdA = 2;
bool LionSharedData::fakeRobots = false;
double LionSharedData::fakeCoef = 1.2;
int LionSharedData::logEveryXGen = 1000;
double LionSharedData::fakeCoefStd = 0;
double LionSharedData::frictionCoef = 0;
double LionSharedData::frictionInflexionPoint = 2.5;
double LionSharedData::tpProba = 1;
double LionSharedData::cost = 5;
bool LionSharedData::optimalPlay = false;
bool LionSharedData::maxTwo = false;
bool LionSharedData::normalCoef = false;
bool LionSharedData::asyncPlay = false;
int LionSharedData::independantCoop = 0;
bool LionSharedData::costAsInput = true;
bool LionSharedData::additiveVar = false;
int LionSharedData::nControl = 0;
double LionSharedData::nOpti = 2;
double LionSharedData::nTolerance = 1.5;




void LionSharedData::initSharedData()
{
    gProperties.checkAndGetPropertyValue("evaluationTime", &LionSharedData::evaluationTime, true);
    gProperties.checkAndGetPropertyValue("nbEvaluationsPerGeneration",
                                         &LionSharedData::nbEvaluationsPerGeneration, true);
    gProperties.checkAndGetPropertyValue("genomeLog", &LionSharedData::genomeLog, true);
    gProperties.checkAndGetPropertyValue("controllerType", &LionSharedData::controllerType, true);
    gProperties.checkAndGetPropertyValue("gMaxTranslationalSpeed", &LionSharedData::maxTranslationalValue, true);
    gProperties.checkAndGetPropertyValue("gMaxRotationalSpeed", &LionSharedData::maxRotationalVelocity, true);
    gProperties.checkAndGetPropertyValue("nbHiddenLayers", &LionSharedData::nbHiddenLayers, true);
    gProperties.checkAndGetPropertyValue("nbNeuronsPerHiddenLayer", &LionSharedData::nbNeuronsPerHiddenLayer,
                                         true);
    gProperties.checkAndGetPropertyValue("takeVideo", &LionSharedData::takeVideo, false);
    gProperties.checkAndGetPropertyValue("oppDecay", &LionSharedData::oppDecay, true);
    gProperties.checkAndGetPropertyValue("maxCoop", &LionSharedData::maxCoop, true);
    gProperties.checkAndGetPropertyValue("b", &LionSharedData::b, true);
    gProperties.checkAndGetPropertyValue("meanA", &LionSharedData::meanA, true);
    gProperties.checkAndGetPropertyValue("stdA", &LionSharedData::stdA, true);
    gProperties.checkAndGetPropertyValue("fakeRobots", &LionSharedData::fakeRobots, true);
    gProperties.checkAndGetPropertyValue("fakeCoef", &LionSharedData::fakeCoef, true);
    gProperties.checkAndGetPropertyValue("logEveryXGen", &LionSharedData::logEveryXGen, false);
    gProperties.checkAndGetPropertyValue("fakeCoefStd", &LionSharedData::fakeCoefStd, false);

    GETVAL(frictionCoef);
    GETVAL(frictionInflexionPoint);
    GETVAL(tpProba);
    GETVAL(optimalPlay);
    GETVAL(cost);
    GETVAL(maxTwo);
    GETVAL(normalCoef);
    GETVAL(asyncPlay);
    GETVAL(independantCoop);
    GETVAL(costAsInput);
    GETVAL(additiveVar);
    GETVAL(nControl);
    GETVAL(nOpti);
    GETVAL(nTolerance);


    assert((maxTwo && nControl == 0) || !maxTwo); // if maxtwo then no ncontrol
}


