/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#include <CoopFixed2/include/CoopFixed2SharedData.h>
#include <CoopFixed2/include/CoopFixed2AnalysisWorldObserver.h>
#include "CoopFixed2/include/CoopFixed2AnalysisOpportunity.h"



#include "RoboroboMain/roborobo.h"
#include "CoopFixed2/include/CoopFixed2AnalysisOpportunity.h"

CoopFixed2AnalysisOpportunity::CoopFixed2AnalysisOpportunity(int __id) : CoopFixed2Opportunity(__id)
{
    setType(10);
    m_nbprev = 0;
    lifeExpectancy = CoopFixed2SharedData::oppDecay;
}


void CoopFixed2AnalysisOpportunity::step() {
    if (m_nbprev > 0 && lifeExpectancy != -1)
    {
        lifeExpectancy--;
    }
    if (lifeExpectancy == 0)
    {
        resetLife();
        auto *wobs = dynamic_cast<CoopFixed2AnalysisWorldObserver*>(gWorld->getWorldObserver());
        wobs->addObjectToTeleport(_id);
    }
    updateColor();
    RoundObject::step();
}

void CoopFixed2AnalysisOpportunity::updateColor()
{
    _displayColorRed = 0;
    _displayColorGreen = static_cast<Uint8>(128 + 127 * m_coop / CoopFixed2SharedData::maxCoop);
    _displayColorBlue = static_cast<Uint8>(128 - 127 * m_coop / CoopFixed2SharedData::maxCoop);
}

void CoopFixed2AnalysisOpportunity::setCoopValue(double coop)
{
    m_coop = coop;
    updateColor();
}

int CoopFixed2AnalysisOpportunity::getNbNearbyRobots() const
{
    return 1 + nearbyRobotIndexes.size();
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

std::string CoopFixed2AnalysisOpportunity::inspect(std::string prefix) {
    std::string info(prefix + "I'm a fake opp\n");
    return info + CoopFixed2Opportunity::inspect(prefix);
}

