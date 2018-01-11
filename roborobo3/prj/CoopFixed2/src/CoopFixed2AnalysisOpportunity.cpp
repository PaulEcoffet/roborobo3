/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#include "CoopFixed2/include/CoopFixed2AnalysisOpportunity.h"



#include "RoboroboMain/roborobo.h"
#include "CoopFixed2/include/CoopFixed2AnalysisOpportunity.h"

CoopFixed2AnalysisOpportunity::CoopFixed2AnalysisOpportunity(int __id) : CoopFixed2Opportunity(__id)
{
    setType(10);
    m_nbprev = 0;
}

void CoopFixed2AnalysisOpportunity::updateColor()
{
    _displayColorRed = 0;
    _displayColorGreen = static_cast<Uint8>(128 + 127 * m_coop / 2);
    _displayColorBlue = static_cast<Uint8>(128 - 127 * m_coop / 2);
}

void CoopFixed2AnalysisOpportunity::setCoopValue(double coop)
{
    m_coop = coop;
    updateColor();
}

int CoopFixed2AnalysisOpportunity::getNbNearbyRobots() const
{
    return 1 + m_nbprev;
}

void CoopFixed2AnalysisOpportunity::isPushed(int id, std::tuple<double, double> speed)
{
    nearbyRobotIndexes.insert(id - gRobotIndexStartOffset);
}

const std::set<int>& CoopFixed2AnalysisOpportunity::getNearbyRobotIndexes() const
{
    return nearbyRobotIndexes;
}

void CoopFixed2AnalysisOpportunity::clearNearbyRobotIndexes()
{
    m_nbprev = nearbyRobotIndexes.size();
    nearbyRobotIndexes.clear();
}

double CoopFixed2AnalysisOpportunity::getCoop() const
{
    return m_coop;
}

void CoopFixed2AnalysisOpportunity::step()
{
    updateColor();
    RoundObject::step();
}

