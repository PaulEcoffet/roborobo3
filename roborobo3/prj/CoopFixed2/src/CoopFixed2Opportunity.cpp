//
// Created by paul on 31/10/17.
//

#include <CoopFixed2/include/CoopFixed2WorldObserver.h>
#include "RoboroboMain/roborobo.h"
#include "core/World/World.h"
#include "CoopFixed2/include/CoopFixed2Opportunity.h"

CoopFixed2Opportunity::CoopFixed2Opportunity(int __id) : RoundObject(__id)
{
    setType(9);
}

int CoopFixed2Opportunity::getNbNearbyRobots() const
{
    return static_cast<int>(robotsOnOppLastTurn.size());
}

void CoopFixed2Opportunity::isPushed(int id, std::tuple<double, double> speed)
{
    int rid = id - gRobotIndexStartOffset;
    if (robotsOnOppLastTurn.size() >= 2 && robotsOnOppLastTurn.count(rid) == 0)
    {
        // The opportunity was full last turn and the new robot was not here before. So we teleport it.
        auto *wobs = dynamic_cast<CoopFixed2WorldObserver*>(gWorld->getWorldObserver());
        wobs->addRobotToTeleport(rid);
    }
    else if (newRobotsOnOppThisTurn.size() < 2)
    {
        newRobotsOnOppThisTurn.insert(rid);
    }
}

const std::set<int>& CoopFixed2Opportunity::getNearbyRobotIndexes() const
{
    return robotsOnOppLastTurn;
}

void CoopFixed2Opportunity::registerNewRobots()
{
    robotsOnOppLastTurn.clear();
    robotsOnOppLastTurn = newRobotsOnOppThisTurn;
    newRobotsOnOppThisTurn.clear();
}


void CoopFixed2Opportunity::step()
{
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
    else if (getNbNearbyRobots() == 1)
    {
        _displayColorRed = 122;
        _displayColorGreen = 164;
        _displayColorBlue = 105;
    }
    else if (getNbNearbyRobots() == 2)
    {
        if (curInv < 1.5)
        {
            _displayColorRed = 189;
            _displayColorGreen = 131;
            _displayColorBlue = 126;
        }
        else if (curInv < 3)
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
    else
    {
        _displayColorRed = 255;
        _displayColorGreen = 0;
        _displayColorBlue = 0;
    }
}

std::string CoopFixed2Opportunity::inspect(std::string prefix)
{
    std::stringstream out;
    out << prefix << "Last inv: " << curInv << ".\n";
    out << prefix << "I have :";
    for (auto index : robotsOnOppLastTurn)
    {
        out << index << ",";
    }
    out << "\n";
    out << prefix << curInv / 4 << "," << (curInv / 4) / 0.5 << ", " << ((curInv /4) - 0.5) / 0.5 << "\n";
    out << prefix << _displayColorRed << "," << _displayColorGreen << "," << _displayColorBlue << "\n";
    return out.str();
}
