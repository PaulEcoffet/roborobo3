//
// Created by paul on 31/10/17.
//

#include "RoboroboMain/roborobo.h"
#include "PartnerControl/include/PartnerControlOpportunity.h"

PartnerControlOpportunity::PartnerControlOpportunity(int __id) : RoundObject(__id)
{
    setType(9);
}

void PartnerControlOpportunity::updateColor()
{
    _displayColorRed = 0;
    _displayColorGreen = static_cast<Uint8>(128 + 127 * m_coop / 2);
    _displayColorBlue = static_cast<Uint8>(128 - 127 * m_coop / 2);
}

void PartnerControlOpportunity::setCoopValue(double coop)
{
    m_coop = coop;
    updateColor();
}

int PartnerControlOpportunity::getNbNearbyRobots() const
{
    return 0;
}

void PartnerControlOpportunity::isPushed(int id, std::tuple<double, double> speed)
{
    nearbyRobotIndexes.insert(id - gRobotIndexStartOffset);
}

const std::set<int> &PartnerControlOpportunity::getNearbyRobotIndexes() const
{
    return nearbyRobotIndexes;
}

void PartnerControlOpportunity::clearNearbyRobotIndexes()
{
    nearbyRobotIndexes.clear();
}

double PartnerControlOpportunity::getCoop() const
{
    return m_coop;
}

void PartnerControlOpportunity::step()
{
    updateColor();
    RoundObject::step();
}

