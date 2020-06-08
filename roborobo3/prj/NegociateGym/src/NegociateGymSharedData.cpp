/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <limits>
#include <RoboroboMain/main.h>
#include "NegociateGym/include/NegociateGymSharedData.h"

#define GETVAL(name)  (gProperties.checkAndGetPropertyValue(#name, &NegociateGymSharedData::name, false))
#define IMPLIES(cond1, cond2) (!cond1 || cond2)


int NegociateGymSharedData::evaluationTime = 1000;
int NegociateGymSharedData::genomeLog = 2000;
int NegociateGymSharedData::controllerType = 0;
double NegociateGymSharedData::maxTranslationalValue = 2;
double NegociateGymSharedData::maxRotationalVelocity = 30;
int NegociateGymSharedData::nbHiddenLayers = 1;
int NegociateGymSharedData::nbNeuronsPerHiddenLayer = 10;
int NegociateGymSharedData::nbEvaluationsPerGeneration = 1;
bool NegociateGymSharedData::takeVideo = true;
int NegociateGymSharedData::seeCoopFromDist = 0;
bool NegociateGymSharedData::prisonerDilemma = false;
bool NegociateGymSharedData::selfAAsInput = true;
int NegociateGymSharedData::oppDecay = -1;
bool NegociateGymSharedData::fixRobotNb = true;
bool NegociateGymSharedData::teleportRobots = false;
double NegociateGymSharedData::maxCoop = 10;
double NegociateGymSharedData::b = 10;
double NegociateGymSharedData::d = 0;
double NegociateGymSharedData::meanA = 5;
double NegociateGymSharedData::stdA = 2;
bool NegociateGymSharedData::tpToNewObj = false;
bool NegociateGymSharedData::totalInvAsInput = true;
bool NegociateGymSharedData::arrivalAsInput = false;
bool NegociateGymSharedData::ownInvAsInput = true;
bool NegociateGymSharedData::onlyOtherInTotalInv = false;
bool NegociateGymSharedData::fakeRobots = false;
int NegociateGymSharedData::memorySize = 10;
double NegociateGymSharedData::fakeCoef = 1.2;
bool NegociateGymSharedData::reputation = true;
int NegociateGymSharedData::logEveryXGen = 1000;
bool NegociateGymSharedData::atLeastTwo = false;
bool NegociateGymSharedData::reverseCoopOutput = false;
bool NegociateGymSharedData::splitNetwork = false;
bool NegociateGymSharedData::randomFakeCoef = false;
double NegociateGymSharedData::fakeCoefStd = 0;
bool NegociateGymSharedData::smartTeleport = false;
bool NegociateGymSharedData::punishment = false;
bool NegociateGymSharedData::punishmentAsInput = false;
bool NegociateGymSharedData::partnerControl = false;
double NegociateGymSharedData::sigma = 0;
bool NegociateGymSharedData::fixCoop = false;
bool NegociateGymSharedData::commonKnowledgeReputation = true;
double NegociateGymSharedData::reputationNoise = 0;
bool NegociateGymSharedData::onlyNforGame = false;
double NegociateGymSharedData::frictionCoef = 0;
double NegociateGymSharedData::frictionInflexionPoint = 2.5;
int NegociateGymSharedData::fitnessUnlockedIter = 0;
double NegociateGymSharedData::tpProba = 1;
bool NegociateGymSharedData::fakeCoefMulSym = true;
int NegociateGymSharedData::proximityTeleport = 0;
int NegociateGymSharedData::nbCluster = 1;
double NegociateGymSharedData::pStayInCluster = -1;
double NegociateGymSharedData::temperature = -1;
bool NegociateGymSharedData::additiveVar = false;
double NegociateGymSharedData::tau = 100.0;
double NegociateGymSharedData::mutRate = 0.01;
double NegociateGymSharedData::mutCoop = 0.1;
bool NegociateGymSharedData::doNotKill = false;
double NegociateGymSharedData::mutProb = 0.0001;
double NegociateGymSharedData::mutProbCoop = 0.01;
double NegociateGymSharedData::mutProbNegociateGym = 0.001;
bool NegociateGymSharedData::putOutOfGame = true;
bool NegociateGymSharedData::wander = false;
bool NegociateGymSharedData::randomObjectPositions = false;


void NegociateGymSharedData::initSharedData()
{
    gProperties.checkAndGetPropertyValue("evaluationTime", &NegociateGymSharedData::evaluationTime, true);
    gProperties.checkAndGetPropertyValue("nbEvaluationsPerGeneration",
                                         &NegociateGymSharedData::nbEvaluationsPerGeneration, true);
    gProperties.checkAndGetPropertyValue("genomeLog", &NegociateGymSharedData::genomeLog, true);
    gProperties.checkAndGetPropertyValue("controllerType", &NegociateGymSharedData::controllerType, true);
    gProperties.checkAndGetPropertyValue("gMaxTranslationalSpeed", &NegociateGymSharedData::maxTranslationalValue, true);
    gProperties.checkAndGetPropertyValue("gMaxRotationalSpeed", &NegociateGymSharedData::maxRotationalVelocity, true);
    gProperties.checkAndGetPropertyValue("nbHiddenLayers", &NegociateGymSharedData::nbHiddenLayers, true);
    gProperties.checkAndGetPropertyValue("nbNeuronsPerHiddenLayer", &NegociateGymSharedData::nbNeuronsPerHiddenLayer,
                                         true);
    gProperties.checkAndGetPropertyValue("takeVideo", &NegociateGymSharedData::takeVideo, false);
    gProperties.checkAndGetPropertyValue("seeCoopFromDist", &NegociateGymSharedData::seeCoopFromDist, false);
    gProperties.checkAndGetPropertyValue("prisonerDilemma", &NegociateGymSharedData::prisonerDilemma, false);
    gProperties.checkAndGetPropertyValue("aAsInput", &NegociateGymSharedData::selfAAsInput, true);
    gProperties.checkAndGetPropertyValue("oppDecay", &NegociateGymSharedData::oppDecay, true);
    gProperties.checkAndGetPropertyValue("fixRobotNb", &NegociateGymSharedData::fixRobotNb, true);
    gProperties.checkAndGetPropertyValue("teleportRobots", &NegociateGymSharedData::teleportRobots, true);
    gProperties.checkAndGetPropertyValue("maxCoop", &NegociateGymSharedData::maxCoop, true);
    gProperties.checkAndGetPropertyValue("b", &NegociateGymSharedData::b, true);
    gProperties.checkAndGetPropertyValue("d", &NegociateGymSharedData::d, true);
    gProperties.checkAndGetPropertyValue("meanA", &NegociateGymSharedData::meanA, true);
    gProperties.checkAndGetPropertyValue("stdA", &NegociateGymSharedData::stdA, true);
    gProperties.checkAndGetPropertyValue("tpToNewObj", &NegociateGymSharedData::tpToNewObj, false);
    gProperties.checkAndGetPropertyValue("totalInvAsInput", &NegociateGymSharedData::totalInvAsInput, true);
    gProperties.checkAndGetPropertyValue("arrivalAsInput", &NegociateGymSharedData::arrivalAsInput, true);
    gProperties.checkAndGetPropertyValue("ownInvAsInput", &NegociateGymSharedData::ownInvAsInput, true);
    gProperties.checkAndGetPropertyValue("onlyOtherInTotalInv", &NegociateGymSharedData::onlyOtherInTotalInv, true);
    gProperties.checkAndGetPropertyValue("memorySize", &NegociateGymSharedData::memorySize, true);
    gProperties.checkAndGetPropertyValue("fakeRobots", &NegociateGymSharedData::fakeRobots, true);
    gProperties.checkAndGetPropertyValue("fakeCoef", &NegociateGymSharedData::fakeCoef, true);
    gProperties.checkAndGetPropertyValue("reputation", &NegociateGymSharedData::reputation, true);
    gProperties.checkAndGetPropertyValue("logEveryXGen", &NegociateGymSharedData::logEveryXGen, false);
    gProperties.checkAndGetPropertyValue("atLeastTwo", &NegociateGymSharedData::atLeastTwo, false);
    gProperties.checkAndGetPropertyValue("reverseCoopOutput", &NegociateGymSharedData::reverseCoopOutput, false);
    gProperties.checkAndGetPropertyValue("splitNetwork", &NegociateGymSharedData::splitNetwork, false);
    gProperties.checkAndGetPropertyValue("randomFakeCoef", &NegociateGymSharedData::randomFakeCoef, false);
    gProperties.checkAndGetPropertyValue("fakeCoefStd", &NegociateGymSharedData::fakeCoefStd, false);
    gProperties.checkAndGetPropertyValue("smartTeleport", &NegociateGymSharedData::smartTeleport, false);
    gProperties.checkAndGetPropertyValue("punishment", &NegociateGymSharedData::punishment, false);
    gProperties.checkAndGetPropertyValue("punishmentAsInput", &NegociateGymSharedData::punishmentAsInput, false);
    gProperties.checkAndGetPropertyValue("partnerControl", &NegociateGymSharedData::partnerControl, false);
    gProperties.checkAndGetPropertyValue("sigma", &NegociateGymSharedData::sigma, false);
    gProperties.checkAndGetPropertyValue("fixCoop", &NegociateGymSharedData::fixCoop, false);
    gProperties.checkAndGetPropertyValue("commonKnowledgeReputation", &NegociateGymSharedData::commonKnowledgeReputation,
                                         false);
    gProperties.checkAndGetPropertyValue("reputationNoise", &NegociateGymSharedData::reputationNoise, false);
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
    GETVAL(additiveVar);
    GETVAL(tau);
    GETVAL(mutRate);
    GETVAL(mutCoop);
    GETVAL(doNotKill);
    GETVAL(mutProb);
    GETVAL(mutProbCoop);
    GETVAL(mutProbNegociateGym);
    GETVAL(putOutOfGame);
    GETVAL(wander);
    GETVAL(randomObjectPositions);

    assert(IMPLIES(wander, !putOutOfGame));
    assert(IMPLIES(wander, randomObjectPositions));
}

#undef INITVAL
#undef GETVAL


