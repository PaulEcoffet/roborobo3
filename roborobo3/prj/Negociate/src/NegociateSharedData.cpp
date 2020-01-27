/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <limits>
#include <RoboroboMain/main.h>
#include "Negociate/include/NegociateSharedData.h"

#define GETVAL(name)  (gProperties.checkAndGetPropertyValue(#name, &NegociateSharedData::name, false))
#define IMPLIES(cond1, cond2) (!cond1 || cond2)


int NegociateSharedData::evaluationTime = 1000;
int NegociateSharedData::genomeLog = 2000;
int NegociateSharedData::controllerType = 0;
double NegociateSharedData::maxTranslationalValue = 2;
double NegociateSharedData::maxRotationalVelocity = 30;
int NegociateSharedData::nbHiddenLayers = 1;
int NegociateSharedData::nbNeuronsPerHiddenLayer = 10;
int NegociateSharedData::nbEvaluationsPerGeneration = 1;
bool NegociateSharedData::takeVideo = true;
int NegociateSharedData::seeCoopFromDist = 0;
bool NegociateSharedData::prisonerDilemma = false;
bool NegociateSharedData::selfAAsInput = true;
int NegociateSharedData::oppDecay = -1;
bool NegociateSharedData::fixRobotNb = true;
bool NegociateSharedData::teleportRobots = false;
double NegociateSharedData::maxCoop = 10;
double NegociateSharedData::b = 10;
double NegociateSharedData::d = 0;
double NegociateSharedData::meanA = 5;
double NegociateSharedData::stdA = 2;
bool NegociateSharedData::tpToNewObj = false;
bool NegociateSharedData::totalInvAsInput = true;
bool NegociateSharedData::arrivalAsInput = false;
bool NegociateSharedData::ownInvAsInput = true;
bool NegociateSharedData::onlyOtherInTotalInv = false;
bool NegociateSharedData::fakeRobots = false;
int NegociateSharedData::memorySize = 10;
double NegociateSharedData::fakeCoef = 1.2;
bool NegociateSharedData::reputation = true;
int NegociateSharedData::logEveryXGen = 1000;
bool NegociateSharedData::atLeastTwo = false;
bool NegociateSharedData::reverseCoopOutput = false;
bool NegociateSharedData::splitNetwork = false;
bool NegociateSharedData::randomFakeCoef = false;
double NegociateSharedData::fakeCoefStd = 0;
bool NegociateSharedData::smartTeleport = false;
bool NegociateSharedData::punishment = false;
bool NegociateSharedData::punishmentAsInput = false;
bool NegociateSharedData::partnerControl = false;
double NegociateSharedData::sigma = 0;
bool NegociateSharedData::fixCoop = false;
bool NegociateSharedData::commonKnowledgeReputation = true;
double NegociateSharedData::reputationNoise = 0;
bool NegociateSharedData::onlyNforGame = false;
double NegociateSharedData::frictionCoef = 0;
double NegociateSharedData::frictionInflexionPoint = 2.5;
int NegociateSharedData::fitnessUnlockedIter = 0;
double NegociateSharedData::tpProba = 1;
bool NegociateSharedData::fakeCoefMulSym = true;
int NegociateSharedData::proximityTeleport = 0;
int NegociateSharedData::nbCluster = 1;
double NegociateSharedData::pStayInCluster = -1;
double NegociateSharedData::temperature = -1;
bool NegociateSharedData::additiveVar = false;
double NegociateSharedData::tau = 100.0;
double NegociateSharedData::mutRate = 0.01;
double NegociateSharedData::mutCoop = 0.1;
bool NegociateSharedData::doNotKill = false;
double NegociateSharedData::mutProb = 0.0001;
double NegociateSharedData::mutProbCoop = 0.01;
double NegociateSharedData::mutProbNegociate = 0.001;
bool NegociateSharedData::putOutOfGame = true;
bool NegociateSharedData::wander = false;
bool NegociateSharedData::randomObjectPositions = false;



void NegociateSharedData::initSharedData()
{
    gProperties.checkAndGetPropertyValue("evaluationTime", &NegociateSharedData::evaluationTime, true);
    gProperties.checkAndGetPropertyValue("nbEvaluationsPerGeneration",
                                         &NegociateSharedData::nbEvaluationsPerGeneration, true);
    gProperties.checkAndGetPropertyValue("genomeLog", &NegociateSharedData::genomeLog, true);
    gProperties.checkAndGetPropertyValue("controllerType", &NegociateSharedData::controllerType, true);
    gProperties.checkAndGetPropertyValue("gMaxTranslationalSpeed", &NegociateSharedData::maxTranslationalValue, true);
    gProperties.checkAndGetPropertyValue("gMaxRotationalSpeed", &NegociateSharedData::maxRotationalVelocity, true);
    gProperties.checkAndGetPropertyValue("nbHiddenLayers", &NegociateSharedData::nbHiddenLayers, true);
    gProperties.checkAndGetPropertyValue("nbNeuronsPerHiddenLayer", &NegociateSharedData::nbNeuronsPerHiddenLayer,
                                         true);
    gProperties.checkAndGetPropertyValue("takeVideo", &NegociateSharedData::takeVideo, false);
    gProperties.checkAndGetPropertyValue("seeCoopFromDist", &NegociateSharedData::seeCoopFromDist, false);
    gProperties.checkAndGetPropertyValue("prisonerDilemma", &NegociateSharedData::prisonerDilemma, false);
    gProperties.checkAndGetPropertyValue("aAsInput", &NegociateSharedData::selfAAsInput, true);
    gProperties.checkAndGetPropertyValue("oppDecay", &NegociateSharedData::oppDecay, true);
    gProperties.checkAndGetPropertyValue("fixRobotNb", &NegociateSharedData::fixRobotNb, true);
    gProperties.checkAndGetPropertyValue("teleportRobots", &NegociateSharedData::teleportRobots, true);
    gProperties.checkAndGetPropertyValue("maxCoop", &NegociateSharedData::maxCoop, true);
    gProperties.checkAndGetPropertyValue("b", &NegociateSharedData::b, true);
    gProperties.checkAndGetPropertyValue("d", &NegociateSharedData::d, true);
    gProperties.checkAndGetPropertyValue("meanA", &NegociateSharedData::meanA, true);
    gProperties.checkAndGetPropertyValue("stdA", &NegociateSharedData::stdA, true);
    gProperties.checkAndGetPropertyValue("tpToNewObj", &NegociateSharedData::tpToNewObj, false);
    gProperties.checkAndGetPropertyValue("totalInvAsInput", &NegociateSharedData::totalInvAsInput, true);
    gProperties.checkAndGetPropertyValue("arrivalAsInput", &NegociateSharedData::arrivalAsInput, true);
    gProperties.checkAndGetPropertyValue("ownInvAsInput", &NegociateSharedData::ownInvAsInput, true);
    gProperties.checkAndGetPropertyValue("onlyOtherInTotalInv", &NegociateSharedData::onlyOtherInTotalInv, true);
    gProperties.checkAndGetPropertyValue("memorySize", &NegociateSharedData::memorySize, true);
    gProperties.checkAndGetPropertyValue("fakeRobots", &NegociateSharedData::fakeRobots, true);
    gProperties.checkAndGetPropertyValue("fakeCoef", &NegociateSharedData::fakeCoef, true);
    gProperties.checkAndGetPropertyValue("reputation", &NegociateSharedData::reputation, true);
    gProperties.checkAndGetPropertyValue("logEveryXGen", &NegociateSharedData::logEveryXGen, false);
    gProperties.checkAndGetPropertyValue("atLeastTwo", &NegociateSharedData::atLeastTwo, false);
    gProperties.checkAndGetPropertyValue("reverseCoopOutput", &NegociateSharedData::reverseCoopOutput, false);
    gProperties.checkAndGetPropertyValue("splitNetwork", &NegociateSharedData::splitNetwork, false);
    gProperties.checkAndGetPropertyValue("randomFakeCoef", &NegociateSharedData::randomFakeCoef, false);
    gProperties.checkAndGetPropertyValue("fakeCoefStd", &NegociateSharedData::fakeCoefStd, false);
    gProperties.checkAndGetPropertyValue("smartTeleport", &NegociateSharedData::smartTeleport, false);
    gProperties.checkAndGetPropertyValue("punishment", &NegociateSharedData::punishment, false);
    gProperties.checkAndGetPropertyValue("punishmentAsInput", &NegociateSharedData::punishmentAsInput, false);
    gProperties.checkAndGetPropertyValue("partnerControl", &NegociateSharedData::partnerControl, false);
    gProperties.checkAndGetPropertyValue("sigma", &NegociateSharedData::sigma, false);
    gProperties.checkAndGetPropertyValue("fixCoop", &NegociateSharedData::fixCoop, false);
    gProperties.checkAndGetPropertyValue("commonKnowledgeReputation", &NegociateSharedData::commonKnowledgeReputation, false);
    gProperties.checkAndGetPropertyValue("reputationNoise", &NegociateSharedData::reputationNoise, false);
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
    GETVAL(mutProbNegociate);
    GETVAL(putOutOfGame);
    GETVAL(wander);
    GETVAL(randomObjectPositions);

    assert(IMPLIES(wander, !putOutOfGame));
    assert(IMPLIES(wander, randomObjectPositions));
}

#undef INITVAL
#undef GETVAL


