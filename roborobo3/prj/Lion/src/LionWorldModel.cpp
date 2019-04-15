/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <RoboroboMain/main.h>
#include "Lion/include/LionWorldModel.h"
#include <boost/algorithm/clamp.hpp>
#include <Lion/include/LionSharedData.h>
#include <Lion/include/LionWorldModel.h>
#include <core/World/World.h>


LionWorldModel::LionWorldModel()
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

void LionWorldModel::setNewSelfA()
{

    selfA = std::max(randgaussian() * LionSharedData::stdA + LionSharedData::meanA, 0.01);
}

double LionWorldModel::meanLastTotalInvest()
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


double LionWorldModel::meanLastOwnInvest()
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

void LionWorldModel::appendOwnInvest(const double invest)
{
    if (lastOwnInvest.size() >= LionSharedData::memorySize)
    {
        lastOwnInvest.pop_front();
    }
    lastOwnInvest.push_back(invest);
}

void LionWorldModel::appendTotalInvest(const double invest)
{
    if (lastTotalInvest.size() >= LionSharedData::memorySize)
    {
        lastTotalInvest.pop_front();
    }
    lastTotalInvest.push_back(invest);
}

void LionWorldModel::appendToCommonKnowledgeReputation(const double d)
{
    if (lastCommonKnowledgeReputation.size() >= 50 /* TODO fix */)
    {
        lastCommonKnowledgeReputation.pop_front();
    }
    lastCommonKnowledgeReputation.push_back(d);

}

double LionWorldModel::meanLastCommonKnowledgeReputation()
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


void LionWorldModel::reset()
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

void LionWorldModel::initOtherReputations()
{
    otherReputations.assign(gInitialNumberOfRobots, 0);
    nbPlays.assign(gInitialNumberOfRobots, 0);
}

void LionWorldModel::updateOtherReputation(int robid, double invest)
{
    const double currep = otherReputations[robid];
    const int n = nbPlays[robid];
    otherReputations[robid] = (currep * n + invest) / (n + 1);
    nbPlays[robid]++;
}

double LionWorldModel::getOtherReputation(int robid)
{
    if (LionSharedData::commonKnowledgeReputation)
    {
        auto *o_wm = dynamic_cast<LionWorldModel *>(gWorld->getRobot(robid)->getWorldModel());
        return o_wm->meanLastCommonKnowledgeReputation();
    }
    return otherReputations[robid];
}

int LionWorldModel::getNbPlays(int robid)
{
    if (LionSharedData::commonKnowledgeReputation)
    {
        return 0;
    }
    return nbPlays[robid];
}

bool LionWorldModel::isPlaying()
{
    return onOpportunity && (arrival <= 2 || !LionSharedData::fixRobotNb) &&
           (!LionSharedData::atLeastTwo || nbOnOpp >= 2);
}

