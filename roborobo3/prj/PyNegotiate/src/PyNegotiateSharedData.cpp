/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <limits>
#include <RoboroboMain/main.h>
#include "PyNegotiate/include/PyNegotiateSharedData.h"

#define GETVAL(name)  (gProperties.checkAndGetPropertyValue(#name, &PyNegotiateSharedData::name, false))
#define IMPLIES(cond1, cond2) (!cond1 || cond2)


int PyNegotiateSharedData::evaluationTime = 1000;
int PyNegotiateSharedData::genomeLog = 2000;
int PyNegotiateSharedData::controllerType = 0;
double PyNegotiateSharedData::maxTranslationalValue = 2;
double PyNegotiateSharedData::maxRotationalVelocity = 30;
int PyNegotiateSharedData::nbHiddenLayers = 1;
int PyNegotiateSharedData::nbNeuronsPerHiddenLayer = 10;
int PyNegotiateSharedData::nbEvaluationsPerGeneration = 1;
bool PyNegotiateSharedData::takeVideo = true;
int PyNegotiateSharedData::seeCoopFromDist = 0;
bool PyNegotiateSharedData::prisonerDilemma = false;
bool PyNegotiateSharedData::selfAAsInput = true;
int PyNegotiateSharedData::oppDecay = -1;
bool PyNegotiateSharedData::fixRobotNb = true;
bool PyNegotiateSharedData::teleportRobots = false;
double PyNegotiateSharedData::maxCoop = 10;
double PyNegotiateSharedData::b = 10;
double PyNegotiateSharedData::d = 0;
double PyNegotiateSharedData::meanA = 5;
double PyNegotiateSharedData::stdA = 2;
bool PyNegotiateSharedData::tpToNewObj = false;
bool PyNegotiateSharedData::totalInvAsInput = true;
bool PyNegotiateSharedData::arrivalAsInput = false;
bool PyNegotiateSharedData::ownInvAsInput = true;
bool PyNegotiateSharedData::onlyOtherInTotalInv = false;
bool PyNegotiateSharedData::fakeRobots = false;
int PyNegotiateSharedData::memorySize = 10;
double PyNegotiateSharedData::fakeCoef = 1.2;
bool PyNegotiateSharedData::reputation = true;
int PyNegotiateSharedData::logEveryXGen = 1000;
bool PyNegotiateSharedData::atLeastTwo = false;
bool PyNegotiateSharedData::reverseCoopOutput = false;
bool PyNegotiateSharedData::splitNetwork = false;
bool PyNegotiateSharedData::randomFakeCoef = false;
double PyNegotiateSharedData::fakeCoefStd = 0;
bool PyNegotiateSharedData::smartTeleport = false;
bool PyNegotiateSharedData::punishment = false;
bool PyNegotiateSharedData::punishmentAsInput = false;
bool PyNegotiateSharedData::partnerControl = false;
double PyNegotiateSharedData::sigma = 0;
bool PyNegotiateSharedData::fixCoop = false;
bool PyNegotiateSharedData::commonKnowledgeReputation = true;
double PyNegotiateSharedData::reputationNoise = 0;
bool PyNegotiateSharedData::onlyNforGame = false;
double PyNegotiateSharedData::frictionCoef = 0;
double PyNegotiateSharedData::frictionInflexionPoint = 2.5;
int PyNegotiateSharedData::fitnessUnlockedIter = 0;
double PyNegotiateSharedData::tpProba = 1;
bool PyNegotiateSharedData::fakeCoefMulSym = true;
int PyNegotiateSharedData::proximityTeleport = 0;
int PyNegotiateSharedData::nbCluster = 1;
double PyNegotiateSharedData::pStayInCluster = -1;
double PyNegotiateSharedData::temperature = -1;
bool PyNegotiateSharedData::additiveVar = false;
double PyNegotiateSharedData::tau = 100.0;
double PyNegotiateSharedData::mutRate = 0.01;
double PyNegotiateSharedData::mutCoop = 0.1;
bool PyNegotiateSharedData::doNotKill = false;
double PyNegotiateSharedData::mutProb = 0.0001;
double PyNegotiateSharedData::mutProbCoop = 0.01;
double PyNegotiateSharedData::mutProbPyNegotiate = 0.001;
bool PyNegotiateSharedData::putOutOfGame = true;
bool PyNegotiateSharedData::wander = false;
bool PyNegotiateSharedData::randomObjectPositions = false;


void PyNegotiateSharedData::initSharedData()
{
    gProperties.checkAndGetPropertyValue("evaluationTime", &PyNegotiateSharedData::evaluationTime, true);
    gProperties.checkAndGetPropertyValue("nbEvaluationsPerGeneration",
                                         &PyNegotiateSharedData::nbEvaluationsPerGeneration, true);
    gProperties.checkAndGetPropertyValue("genomeLog", &PyNegotiateSharedData::genomeLog, true);
    gProperties.checkAndGetPropertyValue("controllerType", &PyNegotiateSharedData::controllerType, true);
    gProperties.checkAndGetPropertyValue("gMaxTranslationalSpeed", &PyNegotiateSharedData::maxTranslationalValue, true);
    gProperties.checkAndGetPropertyValue("gMaxRotationalSpeed", &PyNegotiateSharedData::maxRotationalVelocity, true);
    gProperties.checkAndGetPropertyValue("nbHiddenLayers", &PyNegotiateSharedData::nbHiddenLayers, true);
    gProperties.checkAndGetPropertyValue("nbNeuronsPerHiddenLayer", &PyNegotiateSharedData::nbNeuronsPerHiddenLayer,
                                         true);
    gProperties.checkAndGetPropertyValue("takeVideo", &PyNegotiateSharedData::takeVideo, false);
    gProperties.checkAndGetPropertyValue("seeCoopFromDist", &PyNegotiateSharedData::seeCoopFromDist, false);
    gProperties.checkAndGetPropertyValue("prisonerDilemma", &PyNegotiateSharedData::prisonerDilemma, false);
    gProperties.checkAndGetPropertyValue("aAsInput", &PyNegotiateSharedData::selfAAsInput, true);
    gProperties.checkAndGetPropertyValue("oppDecay", &PyNegotiateSharedData::oppDecay, true);
    gProperties.checkAndGetPropertyValue("fixRobotNb", &PyNegotiateSharedData::fixRobotNb, true);
    gProperties.checkAndGetPropertyValue("teleportRobots", &PyNegotiateSharedData::teleportRobots, true);
    gProperties.checkAndGetPropertyValue("maxCoop", &PyNegotiateSharedData::maxCoop, true);
    gProperties.checkAndGetPropertyValue("b", &PyNegotiateSharedData::b, true);
    gProperties.checkAndGetPropertyValue("d", &PyNegotiateSharedData::d, true);
    gProperties.checkAndGetPropertyValue("meanA", &PyNegotiateSharedData::meanA, true);
    gProperties.checkAndGetPropertyValue("stdA", &PyNegotiateSharedData::stdA, true);
    gProperties.checkAndGetPropertyValue("tpToNewObj", &PyNegotiateSharedData::tpToNewObj, false);
    gProperties.checkAndGetPropertyValue("totalInvAsInput", &PyNegotiateSharedData::totalInvAsInput, true);
    gProperties.checkAndGetPropertyValue("arrivalAsInput", &PyNegotiateSharedData::arrivalAsInput, true);
    gProperties.checkAndGetPropertyValue("ownInvAsInput", &PyNegotiateSharedData::ownInvAsInput, true);
    gProperties.checkAndGetPropertyValue("onlyOtherInTotalInv", &PyNegotiateSharedData::onlyOtherInTotalInv, true);
    gProperties.checkAndGetPropertyValue("memorySize", &PyNegotiateSharedData::memorySize, true);
    gProperties.checkAndGetPropertyValue("fakeRobots", &PyNegotiateSharedData::fakeRobots, true);
    gProperties.checkAndGetPropertyValue("fakeCoef", &PyNegotiateSharedData::fakeCoef, true);
    gProperties.checkAndGetPropertyValue("reputation", &PyNegotiateSharedData::reputation, true);
    gProperties.checkAndGetPropertyValue("logEveryXGen", &PyNegotiateSharedData::logEveryXGen, false);
    gProperties.checkAndGetPropertyValue("atLeastTwo", &PyNegotiateSharedData::atLeastTwo, false);
    gProperties.checkAndGetPropertyValue("reverseCoopOutput", &PyNegotiateSharedData::reverseCoopOutput, false);
    gProperties.checkAndGetPropertyValue("splitNetwork", &PyNegotiateSharedData::splitNetwork, false);
    gProperties.checkAndGetPropertyValue("randomFakeCoef", &PyNegotiateSharedData::randomFakeCoef, false);
    gProperties.checkAndGetPropertyValue("fakeCoefStd", &PyNegotiateSharedData::fakeCoefStd, false);
    gProperties.checkAndGetPropertyValue("smartTeleport", &PyNegotiateSharedData::smartTeleport, false);
    gProperties.checkAndGetPropertyValue("punishment", &PyNegotiateSharedData::punishment, false);
    gProperties.checkAndGetPropertyValue("punishmentAsInput", &PyNegotiateSharedData::punishmentAsInput, false);
    gProperties.checkAndGetPropertyValue("partnerControl", &PyNegotiateSharedData::partnerControl, false);
    gProperties.checkAndGetPropertyValue("sigma", &PyNegotiateSharedData::sigma, false);
    gProperties.checkAndGetPropertyValue("fixCoop", &PyNegotiateSharedData::fixCoop, false);
    gProperties.checkAndGetPropertyValue("commonKnowledgeReputation", &PyNegotiateSharedData::commonKnowledgeReputation,
                                         false);
    gProperties.checkAndGetPropertyValue("reputationNoise", &PyNegotiateSharedData::reputationNoise, false);
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
    GETVAL(mutProbPyNegotiate);
    GETVAL(putOutOfGame);
    GETVAL(wander);
    GETVAL(randomObjectPositions);

    assert(IMPLIES(wander, !putOutOfGame));
    assert(IMPLIES(wander, randomObjectPositions));
}

#undef INITVAL
#undef GETVAL


