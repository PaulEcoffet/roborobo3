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
    return static_cast<int>(robotsOnOppLastTurn.size());
}

void CoopFixed2Opportunity::isPushed(int id, std::tuple<double, double> speed)
{
    int rid = id - gRobotIndexStartOffset;
    if (robotsOnOppLastTurn.size() >= 2 && robotsOnOppLastTurn.count(rid) == 0 && CoopFixed2SharedData::fixRobotNb)
    {
        // The opportunity was full last turn and the new robot was not here before. So we teleport it.
        auto *wobs = dynamic_cast<CoopFixed2WorldObserver*>(gWorld->getWorldObserver());
        wobs->addRobotToTeleport(rid);
    }
    else if (newRobotsOnOppThisTurn.size() < 2 || !CoopFixed2SharedData::fixRobotNb)
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
        if (curInv < 0.4) /* under ESS set to sum_inv = 0.5 */
        {
            _displayColorRed = 189;
            _displayColorGreen = 131;
            _displayColorBlue = 126;
        }
        else if (curInv < 0.8) /* basically playing ESS */
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
    out << prefix << "I have :";
    for (auto index : robotsOnOppLastTurn)
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
