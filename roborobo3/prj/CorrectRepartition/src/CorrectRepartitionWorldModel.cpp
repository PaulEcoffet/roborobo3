/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-31
 */

#include "CorrectRepartition/include/CorrectRepartitionWorldModel.h"

CorrectRepartitionWorldModel::CorrectRepartitionWorldModel()
        : RobotWorldModel(),
          onOpportunity(false)
{
}

double CorrectRepartitionWorldModel::meanLastTotalInvest()
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


double CorrectRepartitionWorldModel::meanLastOwnInvest()
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

void CorrectRepartitionWorldModel::appendOwnInvest(const double invest)
{
    if (lastOwnInvest.size() >= memorySize)
    {
        lastOwnInvest.pop_front();
    }
    lastOwnInvest.push_back(invest);
}

void CorrectRepartitionWorldModel::appendTotalInvest(const double invest)
{
    if (lastTotalInvest.size() >= memorySize)
    {
        lastTotalInvest.pop_front();
    }
    lastTotalInvest.push_back(invest);
}

