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
    if (getNbNearbyRobots() == 0)
    {
        _displayColorRed = 189;
        _displayColorGreen = 131;
        _displayColorBlue = 126;
    }
    else if (getNbNearbyRobots() == 2)
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
    return nearbyRobotIndexes.size();
}

void CorrectRepartitionOpportunity::isPushed(int id, std::tuple<double, double> speed)
{
    const int rid = id - gRobotIndexStartOffset;
    if(std::find(newNearbyRobotIndexes.begin(), newNearbyRobotIndexes.end(), rid) == newNearbyRobotIndexes.end()) { // if not already in the list
        newNearbyRobotIndexes.push_back(rid);
    }
}

const std::vector<int> & CorrectRepartitionOpportunity::getNearbyRobotIndexes() const
{
    return nearbyRobotIndexes;
}

void CorrectRepartitionOpportunity::clearNearbyRobotIndexes()
{
}

void CorrectRepartitionOpportunity::step()
{
    updateColor(); // Update color before clearing the number of robots.
    RoundObject::step();
}

std::string CorrectRepartitionOpportunity::inspect(std::string prefix)
{
    std::stringstream s;
    s << "There is " << getNbNearbyRobots() << " robots on me.\n";
    s << "They are :";
    for (auto rid : getNearbyRobotIndexes())
    {
        s << rid << ", ";
    }
    s << "\n";
    return s.str();
}

void CorrectRepartitionOpportunity::registerNewNearbyRobots() {
    std::vector<int> out;
    // Look for the robot that were here last turn and are still here. Keep the order of arrival.
    for(const auto rid : nearbyRobotIndexes)
    {
        auto pos = std::find(newNearbyRobotIndexes.begin(), newNearbyRobotIndexes.end(), rid);
        if (pos != newNearbyRobotIndexes.end()) // we found something
        {
            out.push_back(rid);
            *pos = -1; /* mark as already taken into account */
        }
    }
    for (const auto rid: newNearbyRobotIndexes)
    {
        if (rid != -1) // This rid hasn't been processed yet
        {
            out.push_back(rid);
        }
    }

    nearbyRobotIndexes = out;
    newNearbyRobotIndexes.clear();
}

