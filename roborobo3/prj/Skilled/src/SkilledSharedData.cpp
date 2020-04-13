/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <limits>
#include <RoboroboMain/main.h>
#include "Skilled/include/SkilledSharedData.h"

#define GETVAL(name)  (gProperties.checkAndGetPropertyValue(#name, &SkilledSharedData::name, false))


int SkilledSharedData::evaluationTime = 1000;
int SkilledSharedData::genomeLog = 2000;
int SkilledSharedData::controllerType = 0;
double SkilledSharedData::maxTranslationalValue = 2;
double SkilledSharedData::maxRotationalVelocity = 30;
int SkilledSharedData::nbHiddenLayers = 1;
int SkilledSharedData::nbNeuronsPerHiddenLayer = 10;
int SkilledSharedData::nbEvaluationsPerGeneration = 1;
bool SkilledSharedData::takeVideo = true;
int SkilledSharedData::oppDecay = -1;
double SkilledSharedData::maxCoop = 10;
double SkilledSharedData::b = 10;
double SkilledSharedData::meanA = 5;
double SkilledSharedData::stdA = 2;
bool SkilledSharedData::fakeRobots = false;
double SkilledSharedData::fakeCoef = 1.2;
int SkilledSharedData::logEveryXGen = 1000;
double SkilledSharedData::fakeCoefStd = 0;
double SkilledSharedData::frictionCoef = 0;
double SkilledSharedData::frictionInflexionPoint = 2.5;
double SkilledSharedData::tpProba = 1;
double SkilledSharedData::cost = 5;
bool SkilledSharedData::optimalPlay = false;
bool SkilledSharedData::maxTwo = false;
bool SkilledSharedData::normalCoef = false;
bool SkilledSharedData::asyncPlay = false;
int SkilledSharedData::independantCoop = 0;
bool SkilledSharedData::costAsInput = true;
bool SkilledSharedData::additiveVar = false;
int SkilledSharedData::nControl = 0;
double SkilledSharedData::nOpti = 2;
double SkilledSharedData::nTolerance = 1.5;
bool SkilledSharedData::hardCoop = false;
bool SkilledSharedData::splitedNbPartInput = true;
bool SkilledSharedData::otherInvAsInput = true;
double SkilledSharedData::normalMut = 0.1;
double SkilledSharedData::mutProb = 0.01;
int SkilledSharedData::maxPlayer = 20;
bool SkilledSharedData::logScore = false;
double SkilledSharedData::skillCost = 0;



void SkilledSharedData::initSharedData() {
    gProperties.checkAndGetPropertyValue("evaluationTime", &SkilledSharedData::evaluationTime, true);
    gProperties.checkAndGetPropertyValue("nbEvaluationsPerGeneration",
                                         &SkilledSharedData::nbEvaluationsPerGeneration, true);
    gProperties.checkAndGetPropertyValue("genomeLog", &SkilledSharedData::genomeLog, true);
    gProperties.checkAndGetPropertyValue("controllerType", &SkilledSharedData::controllerType, true);
    gProperties.checkAndGetPropertyValue("gMaxTranslationalSpeed", &SkilledSharedData::maxTranslationalValue, true);
    gProperties.checkAndGetPropertyValue("gMaxRotationalSpeed", &SkilledSharedData::maxRotationalVelocity, true);
    gProperties.checkAndGetPropertyValue("nbHiddenLayers", &SkilledSharedData::nbHiddenLayers, true);
    gProperties.checkAndGetPropertyValue("nbNeuronsPerHiddenLayer", &SkilledSharedData::nbNeuronsPerHiddenLayer,
                                         true);
    gProperties.checkAndGetPropertyValue("takeVideo", &SkilledSharedData::takeVideo, false);
    gProperties.checkAndGetPropertyValue("oppDecay", &SkilledSharedData::oppDecay, true);
    gProperties.checkAndGetPropertyValue("maxCoop", &SkilledSharedData::maxCoop, true);
    gProperties.checkAndGetPropertyValue("b", &SkilledSharedData::b, true);
    gProperties.checkAndGetPropertyValue("meanA", &SkilledSharedData::meanA, true);
    gProperties.checkAndGetPropertyValue("stdA", &SkilledSharedData::stdA, true);
    gProperties.checkAndGetPropertyValue("fakeRobots", &SkilledSharedData::fakeRobots, true);
    gProperties.checkAndGetPropertyValue("fakeCoef", &SkilledSharedData::fakeCoef, true);
    gProperties.checkAndGetPropertyValue("logEveryXGen", &SkilledSharedData::logEveryXGen, false);
    gProperties.checkAndGetPropertyValue("fakeCoefStd", &SkilledSharedData::fakeCoefStd, false);

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
    GETVAL(hardCoop);
    GETVAL(splitedNbPartInput);
    GETVAL(normalMut);
    GETVAL(mutProb);
    GETVAL(maxPlayer);
    GETVAL(otherInvAsInput);
    GETVAL(logScore);
    GETVAL(skillCost);


    assert((hardCoop && independantCoop == 0 && !fakeRobots) ||
           !hardCoop); // if hardcoop then no independant coop or fake
    assert((maxTwo && nControl == 0) || !maxTwo); // if maxtwo then no ncontrol
}

#undef GETVAL