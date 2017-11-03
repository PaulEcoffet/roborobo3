//
// Created by paul on 31/10/17.
//

#include "RoboroboMain/roborobo.h"
#include "PartnerChoice/include/PartnerChoiceOpportunity.h"

PartnerChoiceOpportunity::PartnerChoiceOpportunity(int __id) : RoundObject(__id)
{
    setType(8);
}

void PartnerChoiceOpportunity::updateColor()
{
    _displayColorRed = 0;
    _displayColorGreen = static_cast<Uint8>(128 + 127 * m_coop / 2);
    _displayColorBlue = static_cast<Uint8>(128 - 127 * m_coop / 2);
}

void PartnerChoiceOpportunity::setCoopValue(double coop)
{
    m_coop = coop;
    updateColor();
}

int PartnerChoiceOpportunity::getNbNearbyRobots() const
{
    return 0;
}

void PartnerChoiceOpportunity::isPushed(int id, std::tuple<double, double> speed)
{
    nearbyRobotIndexes.insert(id - gRobotIndexStartOffset);
}

const std::set<int>& PartnerChoiceOpportunity::getNearbyRobotIndexes() const
{
    return nearbyRobotIndexes;
}

void PartnerChoiceOpportunity::clearNearbyRobotIndexes()
{
    nearbyRobotIndexes.clear();
}

double PartnerChoiceOpportunity::getCoop() const
{
    return m_coop;
}

void PartnerChoiceOpportunity::step()
{
    updateColor();
    RoundObject::step();
}

