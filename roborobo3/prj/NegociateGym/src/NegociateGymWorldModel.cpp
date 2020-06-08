/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2020-05-25
 */

#include "NegociateGymWorldModel.h"
#include "NegociateGym/include/NegociateGymController.h"
#include "neuralnetworks/Elman.h"
#include <set>
#include "World/World.h"
#include "Agents/Agent.h"
#include "WorldModels/RobotWorldModel.h"
#include "Utilities/Misc.h"
#include "neuralnetworks/Perceptron.h"
#include <RoboroboMain/main.h>
#include "NegociateGym/include/NegociateGymWorldModel.h"
#include <boost/algorithm/clamp.hpp>
#include <NegociateGym/include/NegociateGymSharedData.h>
#include <NegociateGym/include/NegociateGymWorldModel.h>
#include <core/World/World.h>
#include <boost/algorithm/clamp.hpp>


std::vector<std::string> NegociateGymController::inputnames;
using boost::algorithm::clamp;


NegociateGymWorldModel::NegociateGymWorldModel()
        : RobotWorldModel(),
          onOpportunity(false),
          selfA(0.5),
          opp(nullptr),
          fake(false),
          fakeCoef(1),
          teleport(false),
          punishment(0),
          spite(0)
{
    setNewSelfA();
    initOtherReputations();
}

void NegociateGymWorldModel::setNewSelfA()
{

    // set the selfA, it makes a non-gaussian ESS distribution though TODO see if skewed is okay
    selfA = std::max(randgaussian() * NegociateGymSharedData::stdA + NegociateGymSharedData::meanA, 0.01);
}

double NegociateGymWorldModel::meanLastTotalInvest()
{
    if (not lastTotalInvest.empty())
    {
        return std::accumulate(lastTotalInvest.begin(), lastTotalInvest.end(), 0.0) / lastTotalInvest.size();
    }
    else
    {
        return 0;
    }
}


double NegociateGymWorldModel::meanLastOwnInvest()
{
    if (not lastOwnInvest.empty())
    {
        return std::accumulate(lastOwnInvest.begin(), lastOwnInvest.end(), 0.0) / lastOwnInvest.size();
    }
    else
    {
        return 0;
    }
}

void NegociateGymWorldModel::appendOwnInvest(const double invest)
{
    if (lastOwnInvest.size() >= NegociateGymSharedData::memorySize)
    {
        lastOwnInvest.pop_front();
    }
    lastOwnInvest.push_back(invest);
}

void NegociateGymWorldModel::appendTotalInvest(const double invest)
{
    if (lastTotalInvest.size() >= NegociateGymSharedData::memorySize)
    {
        lastTotalInvest.pop_front();
    }
    lastTotalInvest.push_back(invest);
}

void NegociateGymWorldModel::appendToCommonKnowledgeReputation(const double d)
{
    if (lastCommonKnowledgeReputation.size() >= 50 /* TODO fix */)
    {
        lastCommonKnowledgeReputation.pop_front();
    }
    lastCommonKnowledgeReputation.push_back(d);

}

double NegociateGymWorldModel::meanLastCommonKnowledgeReputation()
{
    if (not lastCommonKnowledgeReputation.empty())
    {
        return std::accumulate(lastCommonKnowledgeReputation.begin(), lastCommonKnowledgeReputation.end(), 0.0) /
               lastCommonKnowledgeReputation.size();
    }
    else
    {
        return 0;
    }
}


void NegociateGymWorldModel::reset()
{
    lastCommonKnowledgeReputation.clear();
    lastOwnInvest.clear();
    lastTotalInvest.clear();
    onOpportunity = false;
    nbOnOpp = 0;
    arrival = 0;
    teleport = false;
    toBeTeleported = false;
    opp = nullptr;

}

void NegociateGymWorldModel::initOtherReputations()
{
    otherReputations.assign(gInitialNumberOfRobots, 0);
    nbPlays.assign(gInitialNumberOfRobots, 0);
}

void NegociateGymWorldModel::updateOtherReputation(int robid, double invest)
{
    const double currep = otherReputations[robid];
    const int n = nbPlays[robid];
    otherReputations[robid] = (currep * n + invest) / (n + 1);
    nbPlays[robid]++;
}

double NegociateGymWorldModel::getOtherReputation(int robid)
{
    if (NegociateGymSharedData::commonKnowledgeReputation)
    {
        auto *o_wm = dynamic_cast<NegociateGymWorldModel *>(gWorld->getRobot(robid)->getWorldModel());
        return o_wm->meanLastCommonKnowledgeReputation();
    }
    return otherReputations[robid];
}

int NegociateGymWorldModel::getNbPlays(int robid)
{
    if (NegociateGymSharedData::commonKnowledgeReputation)
    {
        return 0;
    }
    return nbPlays[robid];
}

bool NegociateGymWorldModel::isPlaying()
{
    return onOpportunity && (arrival <= 2 || !NegociateGymSharedData::fixRobotNb) &&
           (!NegociateGymSharedData::atLeastTwo || nbOnOpp >= 2);
}

double NegociateGymWorldModel::getCoop(bool trueValue) const
{
    double coop = _cooperationLevel;
    if (!trueValue)
    {
        coop = clamp(coop + fakeCoef, 0, NegociateGymSharedData::maxCoop);
    }
    return coop;
}

const std::vector<double> &NegociateGymWorldModel::getInputs() const
{
    return inputs;
}

void NegociateGymWorldModel::setInputs(const std::vector<double> &inputs)
{
    NegociateGymWorldModel::inputs = inputs;
}

void NegociateGymWorldModel::updateInputs()
{
    size_t i = 0; // input counter
    const int WALL_ID = 0;
    /*
     * Camera inputs
     */
    for (int j = 0; j < _cameraSensorsNb; j++) // camera counter
    {
        bool isOpportunity = false;
        double curnbOnOpp = 0;
        auto entityId = static_cast<int>(getObjectIdFromCameraSensor(j));

        if (entityId >= gPhysicalObjectIndexStartOffset &&
            entityId < gPhysicalObjectIndexStartOffset + gNbOfPhysicalObjects) // is an Object
        {
            auto *opportunity = dynamic_cast<NegociateGymOpportunity *>(
                    gPhysicalObjects[entityId - gPhysicalObjectIndexStartOffset]);
            isOpportunity = true;
            if (opportunity == opp)
            {
                curnbOnOpp = nbOnOpp;
            }
            else
            {
                curnbOnOpp = opportunity->getNbNearbyRobots();
            }
        }
        double dist = getDistanceValueFromCameraSensor(j) / getCameraSensorMaximumDistanceValue(j);
        inputs[i++] = (Agent::isInstanceOf(entityId)) ? dist : 1;
        inputs[i++] = (entityId == WALL_ID) ? dist : 1;
        inputs[i++] = (isOpportunity) ? dist : 1;
        inputs[i++] = curnbOnOpp;
    }
}

const std::vector<double> &NegociateGymWorldModel::getActions() const
{
    return actions;
}

void NegociateGymWorldModel::setActions(const std::vector<double> &actions)
{
    NegociateGymWorldModel::actions = actions;
}

