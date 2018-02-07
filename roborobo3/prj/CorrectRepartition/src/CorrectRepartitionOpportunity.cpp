//
// Created by paul on 31/10/17.
//

#include <CorrectRepartition/include/CorrectRepartitionSharedData.h>
#include "RoboroboMain/roborobo.h"
#include "CorrectRepartition/include/CorrectRepartitionOpportunity.h"

CorrectRepartitionOpportunity::CorrectRepartitionOpportunity(int __id) : RoundObject(__id)
{
    setType(11);
}

void CorrectRepartitionOpportunity::updateColor()
{
    if (prev_nb == 0)
    {
        _displayColorRed = 189;
        _displayColorGreen = 131;
        _displayColorBlue = 126;
    }
    else if (prev_nb == 2)
    {
        _displayColorRed =198;
        _displayColorGreen = 186;
        _displayColorBlue = 58;
    }
    else
    {
        _displayColorRed = 44;
        _displayColorGreen = 83;
        _displayColorBlue = 120;
    }
}

int CorrectRepartitionOpportunity::getNbNearbyRobots() const
{
    return prev_nb;
}

void CorrectRepartitionOpportunity::isPushed(int id, std::tuple<double, double> speed)
{
    nearbyRobotIndexes.insert(id - gRobotIndexStartOffset);
}

const std::set<int>& CorrectRepartitionOpportunity::getNearbyRobotIndexes() const
{
    return nearbyRobotIndexes;
}

void CorrectRepartitionOpportunity::clearNearbyRobotIndexes()
{
    prev_nb = nearbyRobotIndexes.size();
    nearbyRobotIndexes.clear();
}

void CorrectRepartitionOpportunity::step()
{
    updateColor();
    RoundObject::step();
}

std::string CorrectRepartitionOpportunity::inspect(std::string prefix)
{
    std::stringstream s;
    s << "There is " << getNbNearbyRobots() << " robots on me.\n";
    return s.str();
}

