/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <core/RoboroboMain/main.h>
#include "CoopFixed2/include/CoopFixed2WorldModel.h"
#include <boost/algorithm/clamp.hpp>

CoopFixed2WorldModel::CoopFixed2WorldModel()
        : RobotWorldModel(),
          onOpportunity(false),
          cval(0.5)
{
    setNewCval();
}

void CoopFixed2WorldModel::setNewCval() {
    double cvalstd = 0;
    gProperties.checkAndGetPropertyValue("cvalstd", &cvalstd, false);
    cval = boost::algorithm::clamp(randgaussian() * cvalstd + 0.5, 0.0, 1.0);
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

