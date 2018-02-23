//
// Created by paul on 31/10/17.
//

#include <CoopFixed2/include/CoopFixed2WorldObserver.h>
#include <CoopFixed2/include/CoopFixed2SharedData.h>
#include "RoboroboMain/roborobo.h"
#include "core/World/World.h"
#include "CoopFixed2/include/CoopFixed2Opportunity.h"

CoopFixed2Opportunity::CoopFixed2Opportunity(int __id) : RoundObject(__id)
{
    setType(9);
}

int CoopFixed2Opportunity::getNbNearbyRobots() const
{
    return static_cast<int>(nearbyRobotIndexes.size());
}

void CoopFixed2Opportunity::isPushed(int id, std::tuple<double, double> speed)
{
    const int rid = id - gRobotIndexStartOffset;
    if(std::find(newNearbyRobotIndexes.begin(), newNearbyRobotIndexes.end(), rid) == newNearbyRobotIndexes.end()) { // if not already in the list
        newNearbyRobotIndexes.push_back(rid);
    }
}

const std::vector<int>& CoopFixed2Opportunity::getNearbyRobotIndexes() const
{
    return nearbyRobotIndexes;
}

void CoopFixed2Opportunity::registerNewRobots()
{
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


void CoopFixed2Opportunity::step()
{
    int min_nb_robots = (CoopFixed2SharedData::fixRobotNb)? 2 : 1;
    if (lifeExpectancy > 0 && (getNbNearbyRobots() >= min_nb_robots)) {
        lifeExpectancy--;
    }
    else if (lifeExpectancy == 0)
    {
        resetLife();
        auto *wobs = dynamic_cast<CoopFixed2WorldObserver*>(gWorld->getWorldObserver());
        wobs->addObjectToTeleport(_id);
    }
    updateColor();
    RoundObject::step();
}

void CoopFixed2Opportunity::updateColor()
{
    if (getNbNearbyRobots() == 0)
    {
        _displayColorRed = 152;
        _displayColorGreen = 200;
        _displayColorBlue = 158;
    }
    else
    {
        if (curInv < 0.35) /* under ESS set to sum_of_invest = 0.5 */
        {
            _displayColorRed = 189;
            _displayColorGreen = 131;
            _displayColorBlue = 126;
        }
        else if (curInv < 0.7) /* basically playing ESS */
        {
            _displayColorRed =198;
            _displayColorGreen = 186;
            _displayColorBlue = 58;
        }
        else /* Playing above ESS, closer to SO */
        {
            _displayColorRed = 44;
            _displayColorGreen = 83;
            _displayColorBlue = 120;
        }
    }
}

std::string CoopFixed2Opportunity::inspect(std::string prefix)
{
    std::stringstream out;
    out << prefix << "Last inv: " << curInv << ".\n";
    out << prefix << "I have " << getNbNearbyRobots() << " robots on me: ";
    for (auto index : nearbyRobotIndexes)
    {
        out << index << ",";
    }
    out << "\n";
    out << prefix << "Still " << lifeExpectancy << " before I die :'(.\n";
    return out.str();
}

void CoopFixed2Opportunity::resetLife() {
    lifeExpectancy = CoopFixed2SharedData::oppDecay;
}
