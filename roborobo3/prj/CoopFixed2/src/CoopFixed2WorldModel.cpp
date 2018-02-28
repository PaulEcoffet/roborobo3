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
          selfA(0.5)
{
    setNewSelfA();
}

void CoopFixed2WorldModel::setNewSelfA() {

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
    if (lastOwnInvest.size() >= memorySize)
    {
        lastOwnInvest.pop_front();
    }
    lastOwnInvest.push_back(invest);
}

void CoopFixed2WorldModel::appendTotalInvest(const double invest)
{
    if (lastTotalInvest.size() >= memorySize)
    {
        lastTotalInvest.pop_front();
    }
    lastTotalInvest.push_back(invest);
}

