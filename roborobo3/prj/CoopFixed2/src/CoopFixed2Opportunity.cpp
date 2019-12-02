//
// Created by paul on 31/10/17.
//

#include <CoopFixed2/include/CoopFixed2WorldObserver.h>
#include <CoopFixed2/include/CoopFixed2SharedData.h>
#include "RoboroboMain/roborobo.h"
#include "World/World.h"
#include "CoopFixed2/include/CoopFixed2Opportunity.h"

CoopFixed2Opportunity::CoopFixed2Opportunity(int __id) : RoundObject(__id)
{
    setType(9);
    lifeid = __id;
    if (CoopFixed2SharedData::oppDecay > 0)
        lifeExpectancy = 1. / CoopFixed2SharedData::oppDecay;
    else
        lifeExpectancy = 0;
}

int CoopFixed2Opportunity::getNbNearbyRobots() const
{
    return static_cast<int>(nearbyRobotIndexes.size());
}

int CoopFixed2Opportunity::getNbNewNearbyRobots() const
{
    return static_cast<int>(newNearbyRobotIndexes.size());
}

void CoopFixed2Opportunity::isWalked(int id)
{
    const int rid = id - gRobotIndexStartOffset;
    if (std::find(newNearbyRobotIndexes.begin(), newNearbyRobotIndexes.end(), rid) == newNearbyRobotIndexes.end())
    { // if not already in the list
        newNearbyRobotIndexes.push_back(rid);
    }
    //std::cout << "Coucou, c'est " << _id <<", " << rid << " vient d'arriver. On est maintenant " << newNearbyRobotIndexes.size() << "\n";
}

const std::vector<int> &CoopFixed2Opportunity::getNearbyRobotIndexes() const
{
    return nearbyRobotIndexes;
}

void CoopFixed2Opportunity::registerNewRobots()
{
    std::vector<int> out;
    // Look for the robot that were here last turn and are still here. Keep the order of arrival.
    for (const auto rid : nearbyRobotIndexes)
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
    if(random01() < lifeExpectancy)
    {
        kill();
    }
    updateColor();
    RoundObject::step();
}

void CoopFixed2Opportunity::kill()
{
    auto *wobs = dynamic_cast<CoopFixed2WorldObserver *>(gWorld->getWorldObserver());
    nearbyRobotIndexes.clear();
    newNearbyRobotIndexes.clear();
    curA = 0;
    curInv = 0;
    wobs->addObjectToTeleport(_id);
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
        if (curInv < curA * 0.8 - 0.5) /* under xESS = a/n per robot, so for sum over rob xESS = A */
        {
            _displayColorRed = 189;
            _displayColorGreen = 131;
            _displayColorBlue = 126;
        }
        else if (curInv < curA * 1.2 + 1) /* basically playing ESS */
        {
            _displayColorRed = 198;
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
    return out.str();
}
