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
int LionSharedData::seeCoopFromDist = 0;
bool LionSharedData::prisonerDilemma = false;
bool LionSharedData::selfAAsInput = true;
int LionSharedData::oppDecay = -1;
bool LionSharedData::fixRobotNb = true;
bool LionSharedData::teleportRobots = false;
double LionSharedData::maxCoop = 10;
double LionSharedData::b = 10;
double LionSharedData::d = 0;
double LionSharedData::meanA = 5;
double LionSharedData::stdA = 2;
bool LionSharedData::tpToNewObj = false;
bool LionSharedData::totalInvAsInput = true;
bool LionSharedData::arrivalAsInput = false;
bool LionSharedData::ownInvAsInput = true;
bool LionSharedData::onlyOtherInTotalInv = false;
bool LionSharedData::fakeRobots = false;
int LionSharedData::memorySize = 10;
double LionSharedData::fakeCoef = 1.2;
bool LionSharedData::reputation = true;
int LionSharedData::logEveryXGen = 1000;
bool LionSharedData::atLeastTwo = false;
bool LionSharedData::reverseCoopOutput = false;
bool LionSharedData::splitNetwork = false;
bool LionSharedData::randomFakeCoef = false;
double LionSharedData::fakeCoefStd = 0;
bool LionSharedData::smartTeleport = false;
bool LionSharedData::punishment = false;
bool LionSharedData::punishmentAsInput = false;
bool LionSharedData::partnerControl = false;
double LionSharedData::sigma = 0;
bool LionSharedData::fixCoop = false;
bool LionSharedData::commonKnowledgeReputation = true;
double LionSharedData::reputationNoise = 0;
bool LionSharedData::onlyNforGame = false;
double LionSharedData::frictionCoef = 0;
double LionSharedData::frictionInflexionPoint = 2.5;
int LionSharedData::fitnessUnlockedIter = 0;
double LionSharedData::tpProba = 1;
bool LionSharedData::fakeCoefMulSym = true;
int LionSharedData::proximityTeleport = 0;
int LionSharedData::nbCluster = 1;
double LionSharedData::pStayInCluster = -1;
double LionSharedData::temperature = -1;

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
    gProperties.checkAndGetPropertyValue("seeCoopFromDist", &LionSharedData::seeCoopFromDist, false);
    gProperties.checkAndGetPropertyValue("prisonerDilemma", &LionSharedData::prisonerDilemma, false);
    gProperties.checkAndGetPropertyValue("aAsInput", &LionSharedData::selfAAsInput, true);
    gProperties.checkAndGetPropertyValue("oppDecay", &LionSharedData::oppDecay, true);
    gProperties.checkAndGetPropertyValue("fixRobotNb", &LionSharedData::fixRobotNb, true);
    gProperties.checkAndGetPropertyValue("teleportRobots", &LionSharedData::teleportRobots, true);
    gProperties.checkAndGetPropertyValue("maxCoop", &LionSharedData::maxCoop, true);
    gProperties.checkAndGetPropertyValue("b", &LionSharedData::b, true);
    gProperties.checkAndGetPropertyValue("d", &LionSharedData::d, true);
    gProperties.checkAndGetPropertyValue("meanA", &LionSharedData::meanA, true);
    gProperties.checkAndGetPropertyValue("stdA", &LionSharedData::stdA, true);
    gProperties.checkAndGetPropertyValue("tpToNewObj", &LionSharedData::tpToNewObj, false);
    gProperties.checkAndGetPropertyValue("totalInvAsInput", &LionSharedData::totalInvAsInput, true);
    gProperties.checkAndGetPropertyValue("arrivalAsInput", &LionSharedData::arrivalAsInput, true);
    gProperties.checkAndGetPropertyValue("ownInvAsInput", &LionSharedData::ownInvAsInput, true);
    gProperties.checkAndGetPropertyValue("onlyOtherInTotalInv", &LionSharedData::onlyOtherInTotalInv, true);
    gProperties.checkAndGetPropertyValue("memorySize", &LionSharedData::memorySize, true);
    gProperties.checkAndGetPropertyValue("fakeRobots", &LionSharedData::fakeRobots, true);
    gProperties.checkAndGetPropertyValue("fakeCoef", &LionSharedData::fakeCoef, true);
    gProperties.checkAndGetPropertyValue("reputation", &LionSharedData::reputation, true);
    gProperties.checkAndGetPropertyValue("logEveryXGen", &LionSharedData::logEveryXGen, false);
    gProperties.checkAndGetPropertyValue("atLeastTwo", &LionSharedData::atLeastTwo, false);
    gProperties.checkAndGetPropertyValue("reverseCoopOutput", &LionSharedData::reverseCoopOutput, false);
    gProperties.checkAndGetPropertyValue("splitNetwork", &LionSharedData::splitNetwork, false);
    gProperties.checkAndGetPropertyValue("randomFakeCoef", &LionSharedData::randomFakeCoef, false);
    gProperties.checkAndGetPropertyValue("fakeCoefStd", &LionSharedData::fakeCoefStd, false);
    gProperties.checkAndGetPropertyValue("smartTeleport", &LionSharedData::smartTeleport, false);
    gProperties.checkAndGetPropertyValue("punishment", &LionSharedData::punishment, false);
    gProperties.checkAndGetPropertyValue("punishmentAsInput", &LionSharedData::punishmentAsInput, false);
    gProperties.checkAndGetPropertyValue("partnerControl", &LionSharedData::partnerControl, false);
    gProperties.checkAndGetPropertyValue("sigma", &LionSharedData::sigma, false);
    gProperties.checkAndGetPropertyValue("fixCoop", &LionSharedData::fixCoop, false);
    gProperties.checkAndGetPropertyValue("commonKnowledgeReputation", &LionSharedData::commonKnowledgeReputation,
                                         false);
    gProperties.checkAndGetPropertyValue("reputationNoise", &LionSharedData::reputationNoise, false);
    GETVAL(onlyNforGame);
    GETVAL(frictionCoef);
    GETVAL(frictionInflexionPoint);
    GETVAL(fitnessUnlockedIter);
    GETVAL(tpProba);
    GETVAL(fakeCoefMulSym);
    GETVAL(proximityTeleport);
    GETVAL(nbCluster);
    GETVAL(pStayInCluster);
    GETVAL(temperature);
}


