//
// Created by paul on 31/10/17.
//

#include <Negociate/include/NegociateWorldObserver.h>
#include <Negociate/include/NegociateSharedData.h>
#include "RoboroboMain/roborobo.h"
#include "World/World.h"
#include "Negociate/include/NegociateOpportunity.h"

NegociateOpportunity::NegociateOpportunity(int __id) : RoundObject(__id), touchingrobotsSet()
{
    setType(9);
    lifeid = __id;
    if (NegociateSharedData::oppDecay > 0)
        lifeExpectancy = 1. / NegociateSharedData::oppDecay;
    else
        lifeExpectancy = 0;
}

int NegociateOpportunity::getNbNearbyRobots() const
{
    return static_cast<int>(nearbyRobotIndexes.size());
}

int NegociateOpportunity::getNbNewNearbyRobots() const
{
    return static_cast<int>(newNearbyRobotIndexes.size());
}

void NegociateOpportunity::isWalked(int id)
{
    const int rid = id - gRobotIndexStartOffset;
    if (std::find(newNearbyRobotIndexes.begin(), newNearbyRobotIndexes.end(), rid) ==
        newNearbyRobotIndexes.end())
    { // if not already in the list
        newNearbyRobotIndexes.push_back(rid);
    }
    //std::cout << "Coucou, c'est " << _id <<", " << rid << " vient d'arriver. On est maintenant " << newNearbyRobotIndexes.size() << "\n";
}

const std::vector<int> &NegociateOpportunity::getNearbyRobotIndexes() const
{
    return nearbyRobotIndexes;
}

void NegociateOpportunity::registerNewRobots()
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


void NegociateOpportunity::step()
{
    touchingrobotsSet.clear();
    if (random01() < lifeExpectancy)
    {
        kill();
    }
    updateColor();
    RoundObject::step();
}

void NegociateOpportunity::kill()
{
    auto *wobs = dynamic_cast<NegociateWorldObserver *>(gWorld->getWorldObserver());
    nearbyRobotIndexes.clear();
    newNearbyRobotIndexes.clear();
    curA = 0;
    curInv = 0;
    wobs->addObjectToTeleport(_id);
}

void NegociateOpportunity::updateColor()
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

std::string NegociateOpportunity::inspect(std::string prefix)
{
    std::stringstream out;
    out << prefix << "I'm a negociate opportunity !\n";
    out << prefix << "Last inv: " << curInv << ".\n";
    out << prefix << "I have " << getNbNearbyRobots() << " robots on me: ";
    for (auto index : nearbyRobotIndexes)
    {
        out << index << ",";
    }
    out << "\n";
    return out.str();
}

void NegociateOpportunity::isPushed(int rid, std::tuple<double, double> speed)
{
    //std::cout << rid << " touched me (" << _id << ").\n";
    touchingrobotsSet.emplace(rid - gRobotIndexStartOffset);
}

bool NegociateOpportunity::hasBeenTouched(int rid) const
{
    return touchingrobotsSet.find(rid) != touchingrobotsSet.end();
}
