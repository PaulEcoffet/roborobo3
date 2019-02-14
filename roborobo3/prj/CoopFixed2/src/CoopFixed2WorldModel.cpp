/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <RoboroboMain/main.h>
#include "CoopFixed2/include/CoopFixed2WorldModel.h"
#include <boost/algorithm/clamp.hpp>
#include <CoopFixed2/include/CoopFixed2SharedData.h>
#include <CoopFixed2/include/CoopFixed2WorldModel.h>
#include <core/World/World.h>


CoopFixed2WorldModel::CoopFixed2WorldModel()
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

void CoopFixed2WorldModel::setNewSelfA()
{

    // set the selfA, it makes a non-gaussian ESS distribution though TODO see if skewed is okay
    selfA = std::max(randgaussian() * CoopFixed2SharedData::stdA + CoopFixed2SharedData::meanA, 0.01);
}

double CoopFixed2WorldModel::meanLastTotalInvest()
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


double CoopFixed2WorldModel::meanLastOwnInvest()
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

void CoopFixed2WorldModel::appendOwnInvest(const double invest)
{
    if (lastOwnInvest.size() >= CoopFixed2SharedData::memorySize)
    {
        lastOwnInvest.pop_front();
    }
    lastOwnInvest.push_back(invest);
}

void CoopFixed2WorldModel::appendTotalInvest(const double invest)
{
    if (lastTotalInvest.size() >= CoopFixed2SharedData::memorySize)
    {
        lastTotalInvest.pop_front();
    }
    lastTotalInvest.push_back(invest);
}

void CoopFixed2WorldModel::appendToCommonKnowledgeReputation(const double d)
{
    if (lastCommonKnowledgeReputation.size() >= 50 /* TODO fix */)
    {
        lastCommonKnowledgeReputation.pop_front();
    }
    lastCommonKnowledgeReputation.push_back(d);

}

double CoopFixed2WorldModel::meanLastCommonKnowledgeReputation()
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


void CoopFixed2WorldModel::reset()
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

void CoopFixed2WorldModel::initOtherReputations()
{
    otherReputations.assign(gInitialNumberOfRobots, 0);
    nbPlays.assign(gInitialNumberOfRobots, 0);
}

void CoopFixed2WorldModel::updateOtherReputation(int robid, double invest)
{
    const double currep = otherReputations[robid];
    const int n = nbPlays[robid];
    otherReputations[robid] = (currep * n + invest) / (n+1);
    nbPlays[robid]++;
}

double CoopFixed2WorldModel::getOtherReputation(int robid)
{
    if (CoopFixed2SharedData::commonKnowledgeReputation)
    {
        auto *o_wm = dynamic_cast<CoopFixed2WorldModel*>(gWorld->getRobot(robid)->getWorldModel());
        return o_wm->meanLastCommonKnowledgeReputation();
    }
    return otherReputations[robid];
}

int CoopFixed2WorldModel::getNbPlays(int robid)
{
    if (CoopFixed2SharedData::commonKnowledgeReputation)
    {
        return 0;
    }
    return nbPlays[robid];
}

bool CoopFixed2WorldModel::isPlaying()
{
    return onOpportunity && (arrival <= 2 || !CoopFixed2SharedData::fixRobotNb) && (!CoopFixed2SharedData::atLeastTwo || nbOnOpp >= 2);
}

