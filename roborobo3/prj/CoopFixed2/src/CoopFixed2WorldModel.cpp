/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <RoboroboMain/main.h>
#include "CoopFixed2/include/CoopFixed2WorldModel.h"
#include <boost/algorithm/clamp.hpp>
#include <CoopFixed2/include/CoopFixed2SharedData.h>

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

void CoopFixed2WorldModel::appendToReputation(const double d)
{
    if (lastReputation.size() >= 50 /* TODO fix */)
    {
        lastReputation.pop_front();
    }
    lastReputation.push_back(d);

}

double CoopFixed2WorldModel::meanLastReputation()
{
    if (not lastReputation.empty())
    {
        return std::accumulate(lastReputation.begin(), lastReputation.end(), 0.0) / lastReputation.size();
    }
    else
    {
        return 0;
    }
}

void CoopFixed2WorldModel::reset()
{
    lastReputation.clear();
    lastOwnInvest.clear();
    lastTotalInvest.clear();
    onOpportunity = false;
    nbOnOpp = 0;
    arrival = 0;
    opp = nullptr;

}

