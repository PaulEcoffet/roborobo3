/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <limits>
#include <RoboroboMain/main.h>
#include "CoopFixed2/include/CoopFixed2SharedData.h"

#define GETVAL(name)  (gProperties.checkAndGetPropertyValue(#name, &CoopFixed2SharedData::name, false))



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
double CoopFixed2SharedData::d = 0;
double CoopFixed2SharedData::meanA = 5;
double CoopFixed2SharedData::stdA = 2;
bool CoopFixed2SharedData::tpToNewObj = false;
bool CoopFixed2SharedData::totalInvAsInput = true;
bool CoopFixed2SharedData::arrivalAsInput = false;
bool CoopFixed2SharedData::ownInvAsInput = true;
bool CoopFixed2SharedData::onlyOtherInTotalInv = false;
bool CoopFixed2SharedData::fakeRobots = false;
int CoopFixed2SharedData::memorySize = 10;
double CoopFixed2SharedData::fakeCoef = 1.2;
bool CoopFixed2SharedData::reputation = true;
int CoopFixed2SharedData::logEveryXGen = 1000;
bool CoopFixed2SharedData::atLeastTwo = false;
bool CoopFixed2SharedData::reverseCoopOutput = false;
bool CoopFixed2SharedData::splitNetwork = false;
bool CoopFixed2SharedData::randomFakeCoef = false;
double CoopFixed2SharedData::fakeCoefStd = 0;
bool CoopFixed2SharedData::smartTeleport = false;
bool CoopFixed2SharedData::punishment = false;
bool CoopFixed2SharedData::punishmentAsInput = false;
bool CoopFixed2SharedData::partnerControl = false;
double CoopFixed2SharedData::sigma = 0;
bool CoopFixed2SharedData::fixCoop = false;
bool CoopFixed2SharedData::commonKnowledgeReputation = true;
double CoopFixed2SharedData::reputationNoise = 0;
bool CoopFixed2SharedData::onlyNforGame = false;
double CoopFixed2SharedData::frictionCoef = 0;
double CoopFixed2SharedData::frictionInflexionPoint = 2.5;
int CoopFixed2SharedData::fitnessUnlockedIter = 0;
double CoopFixed2SharedData::tpProba = 1;
bool CoopFixed2SharedData::fakeCoefMulSym = true;


void CoopFixed2SharedData::initSharedData()
{
    gProperties.checkAndGetPropertyValue("evaluationTime", &CoopFixed2SharedData::evaluationTime, true);
    gProperties.checkAndGetPropertyValue("nbEvaluationsPerGeneration",
                                         &CoopFixed2SharedData::nbEvaluationsPerGeneration, true);
    gProperties.checkAndGetPropertyValue("genomeLog", &CoopFixed2SharedData::genomeLog, true);
    gProperties.checkAndGetPropertyValue("controllerType", &CoopFixed2SharedData::controllerType, true);
    gProperties.checkAndGetPropertyValue("gMaxTranslationalSpeed", &CoopFixed2SharedData::maxTranslationalValue, true);
    gProperties.checkAndGetPropertyValue("gMaxRotationalSpeed", &CoopFixed2SharedData::maxRotationalVelocity, true);
    gProperties.checkAndGetPropertyValue("nbHiddenLayers", &CoopFixed2SharedData::nbHiddenLayers, true);
    gProperties.checkAndGetPropertyValue("nbNeuronsPerHiddenLayer", &CoopFixed2SharedData::nbNeuronsPerHiddenLayer,
                                         true);
    gProperties.checkAndGetPropertyValue("takeVideo", &CoopFixed2SharedData::takeVideo, false);
    gProperties.checkAndGetPropertyValue("seeCoopFromDist", &CoopFixed2SharedData::seeCoopFromDist, false);
    gProperties.checkAndGetPropertyValue("prisonerDilemma", &CoopFixed2SharedData::prisonerDilemma, false);
    gProperties.checkAndGetPropertyValue("aAsInput", &CoopFixed2SharedData::selfAAsInput, true);
    gProperties.checkAndGetPropertyValue("oppDecay", &CoopFixed2SharedData::oppDecay, true);
    gProperties.checkAndGetPropertyValue("fixRobotNb", &CoopFixed2SharedData::fixRobotNb, true);
    gProperties.checkAndGetPropertyValue("teleportRobots", &CoopFixed2SharedData::teleportRobots, true);
    gProperties.checkAndGetPropertyValue("maxCoop", &CoopFixed2SharedData::maxCoop, true);
    gProperties.checkAndGetPropertyValue("b", &CoopFixed2SharedData::b, true);
    gProperties.checkAndGetPropertyValue("d", &CoopFixed2SharedData::d, true);
    gProperties.checkAndGetPropertyValue("meanA", &CoopFixed2SharedData::meanA, true);
    gProperties.checkAndGetPropertyValue("stdA", &CoopFixed2SharedData::stdA, true);
    gProperties.checkAndGetPropertyValue("tpToNewObj", &CoopFixed2SharedData::tpToNewObj, false);
    gProperties.checkAndGetPropertyValue("totalInvAsInput", &CoopFixed2SharedData::totalInvAsInput, true);
    gProperties.checkAndGetPropertyValue("arrivalAsInput", &CoopFixed2SharedData::arrivalAsInput, true);
    gProperties.checkAndGetPropertyValue("ownInvAsInput", &CoopFixed2SharedData::ownInvAsInput, true);
    gProperties.checkAndGetPropertyValue("onlyOtherInTotalInv", &CoopFixed2SharedData::onlyOtherInTotalInv, true);
    gProperties.checkAndGetPropertyValue("memorySize", &CoopFixed2SharedData::memorySize, true);
    gProperties.checkAndGetPropertyValue("fakeRobots", &CoopFixed2SharedData::fakeRobots, true);
    gProperties.checkAndGetPropertyValue("fakeCoef", &CoopFixed2SharedData::fakeCoef, true);
    gProperties.checkAndGetPropertyValue("reputation", &CoopFixed2SharedData::reputation, true);
    gProperties.checkAndGetPropertyValue("logEveryXGen", &CoopFixed2SharedData::logEveryXGen, false);
    gProperties.checkAndGetPropertyValue("atLeastTwo", &CoopFixed2SharedData::atLeastTwo, false);
    gProperties.checkAndGetPropertyValue("reverseCoopOutput", &CoopFixed2SharedData::reverseCoopOutput, false);
    gProperties.checkAndGetPropertyValue("splitNetwork", &CoopFixed2SharedData::splitNetwork, false);
    gProperties.checkAndGetPropertyValue("randomFakeCoef", &CoopFixed2SharedData::randomFakeCoef, false);
    gProperties.checkAndGetPropertyValue("fakeCoefStd", &CoopFixed2SharedData::fakeCoefStd, false);
    gProperties.checkAndGetPropertyValue("smartTeleport", &CoopFixed2SharedData::smartTeleport, false);
    gProperties.checkAndGetPropertyValue("punishment", &CoopFixed2SharedData::punishment, false);
    gProperties.checkAndGetPropertyValue("punishmentAsInput", &CoopFixed2SharedData::punishmentAsInput, false);
    gProperties.checkAndGetPropertyValue("partnerControl", &CoopFixed2SharedData::partnerControl, false);
    gProperties.checkAndGetPropertyValue("sigma", &CoopFixed2SharedData::sigma, false);
    gProperties.checkAndGetPropertyValue("fixCoop", &CoopFixed2SharedData::fixCoop, false);
    gProperties.checkAndGetPropertyValue("commonKnowledgeReputation", &CoopFixed2SharedData::commonKnowledgeReputation, false);
    gProperties.checkAndGetPropertyValue("reputationNoise", &CoopFixed2SharedData::reputationNoise, false);
    GETVAL(onlyNforGame);
    GETVAL(frictionCoef);
    GETVAL(frictionInflexionPoint);
    GETVAL(fitnessUnlockedIter);
    GETVAL(tpProba);
    GETVAL(fakeCoefMulSym);
}


