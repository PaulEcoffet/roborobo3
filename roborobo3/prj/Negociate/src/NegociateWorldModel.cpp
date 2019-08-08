/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <RoboroboMain/main.h>
#include "Negociate/include/NegociateWorldModel.h"
#include <boost/algorithm/clamp.hpp>
#include <Negociate/include/NegociateSharedData.h>
#include <Negociate/include/NegociateWorldModel.h>
#include <core/World/World.h>
#include <boost/algorithm/clamp.hpp>


using boost::algorithm::clamp;


NegociateWorldModel::NegociateWorldModel()
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

void NegociateWorldModel::setNewSelfA()
{

    // set the selfA, it makes a non-gaussian ESS distribution though TODO see if skewed is okay
    selfA = std::max(randgaussian() * NegociateSharedData::stdA + NegociateSharedData::meanA, 0.01);
}

double NegociateWorldModel::meanLastTotalInvest()
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


double NegociateWorldModel::meanLastOwnInvest()
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

void NegociateWorldModel::appendOwnInvest(const double invest)
{
    if (lastOwnInvest.size() >= NegociateSharedData::memorySize)
    {
        lastOwnInvest.pop_front();
    }
    lastOwnInvest.push_back(invest);
}

void NegociateWorldModel::appendTotalInvest(const double invest)
{
    if (lastTotalInvest.size() >= NegociateSharedData::memorySize)
    {
        lastTotalInvest.pop_front();
    }
    lastTotalInvest.push_back(invest);
}

void NegociateWorldModel::appendToCommonKnowledgeReputation(const double d)
{
    if (lastCommonKnowledgeReputation.size() >= 50 /* TODO fix */)
    {
        lastCommonKnowledgeReputation.pop_front();
    }
    lastCommonKnowledgeReputation.push_back(d);

}

double NegociateWorldModel::meanLastCommonKnowledgeReputation()
{
    if (not lastCommonKnowledgeReputation.empty())
    {
        return std::accumulate(lastCommonKnowledgeReputation.begin(), lastCommonKnowledgeReputation.end(), 0.0) / lastCommonKnowledgeReputation.size();
    }
    else
    {
        return 0;
    }
}


void NegociateWorldModel::reset()
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

void NegociateWorldModel::initOtherReputations()
{
    otherReputations.assign(gInitialNumberOfRobots, 0);
    nbPlays.assign(gInitialNumberOfRobots, 0);
}

void NegociateWorldModel::updateOtherReputation(int robid, double invest)
{
    const double currep = otherReputations[robid];
    const int n = nbPlays[robid];
    otherReputations[robid] = (currep * n + invest) / (n+1);
    nbPlays[robid]++;
}

double NegociateWorldModel::getOtherReputation(int robid)
{
    if (NegociateSharedData::commonKnowledgeReputation)
    {
        auto *o_wm = dynamic_cast<NegociateWorldModel*>(gWorld->getRobot(robid)->getWorldModel());
        return o_wm->meanLastCommonKnowledgeReputation();
    }
    return otherReputations[robid];
}

int NegociateWorldModel::getNbPlays(int robid)
{
    if (NegociateSharedData::commonKnowledgeReputation)
    {
        return 0;
    }
    return nbPlays[robid];
}

bool NegociateWorldModel::isPlaying()
{
    return onOpportunity && (arrival <= 2 || !NegociateSharedData::fixRobotNb) && (!NegociateSharedData::atLeastTwo || nbOnOpp >= 2);
}

double NegociateWorldModel::getCoop(bool trueValue) const
{
    double coop = _cooperationLevel;
    if (!trueValue)
    {
        coop = clamp(coop + fakeCoef, 0, NegociateSharedData::maxCoop);
    }
    return coop;
}

