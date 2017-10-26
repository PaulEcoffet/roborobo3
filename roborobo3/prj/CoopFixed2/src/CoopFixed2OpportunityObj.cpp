/*
 * CoopFixed2OppotunityObj.cpp
 *
 *  Created on: 9 oct. 2017
 *      Author: Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 */

#include "CoopFixed2/include/CoopFixed2OpportunityObj.h"
#include "CoopFixed2/include/CoopFixed2SharedData.h"
#include "RoboroboMain/roborobo.h"
#include "World/World.h"

CoopFixed2OpportunityObj::CoopFixed2OpportunityObj(int __id): RoundObject(__id)
{
    setType(7);

}

CoopFixed2OpportunityObj::~CoopFixed2OpportunityObj()
{
}

void CoopFixed2OpportunityObj::step()
{
    if (_prevNearbyRobots.size() == 0)
    {
        _displayColorBlue = 0xFF;
        _displayColorGreen = 0x00;
        _displayColorRed = 0x00;
    }
    else if(_prevNearbyRobots.size() == 1)
    {
        _displayColorBlue = 0xFF;
        _displayColorGreen = 0xFF;
        _displayColorRed = 0x00;

    }
    else if (_prevNearbyRobots.size() == 2)
    {
        _displayColorBlue = 0x00;
        _displayColorGreen = 0xFF - _lockRemainingTime * 10;
        _displayColorRed = 0x00 + _lockRemainingTime * 10;
    }
    else
    {
        _displayColorBlue = 0x00;
        _displayColorGreen = 0x00;
        _displayColorRed = 0xFF;
    }
    RoundObject::step();
}

void CoopFixed2OpportunityObj::isPushed( int __idAgent, std::tuple<double, double> __speed )
{
    /*
     * Ensure that no new agent comes. If at the previous turn, the opportunity was full and the agent was not here
     * before, it is teleported far away.
     * Therefore, an agent cannot come at the same turn as an agent leaves. It simplifies greatly the readability of
     * the code
     */
    const auto indexAgent = __idAgent - gRobotIndexStartOffset;
    auto worldObserver = dynamic_cast<CoopFixed2WorldObserver *>(gWorld->getWorldObserver());
    if (_prevNearbyRobots.size() == 2 and _prevNearbyRobots.count(indexAgent) == 0)
    {
        worldObserver->addRobotToTeleport(indexAgent);
        return;
    }

    // We can safely add the robot now. We know the opportunity is not full or the robot was already present there.
    // We don't have to deal with the fact that a robot who was already there last turn will arrive later on.
    if (_curNearbyRobots.size() < 2)
    {
        _curNearbyRobots.insert(indexAgent);
    }
}

std::set<int> CoopFixed2OpportunityObj::getNearbyRobots()
{
    return _curNearbyRobots;
}

int CoopFixed2OpportunityObj::getLockRemainingTime() const
{
    return _lockRemainingTime;
}

void CoopFixed2OpportunityObj::setLockRemainingTime(int lockRemainingTime)
{
    _lockRemainingTime = lockRemainingTime;
}

void CoopFixed2OpportunityObj::decrementLockRemainingTime()
{
    if (_lockRemainingTime > 0)
    {
        _lockRemainingTime--;
    }
}

void CoopFixed2OpportunityObj::clearNearbyRobots()
{
    _prevNearbyRobots.clear();
    _prevNearbyRobots.insert(_curNearbyRobots.begin(), _curNearbyRobots.end());
    _curNearbyRobots.clear();
}

int CoopFixed2OpportunityObj::getNbNearbyRobots()
{
    return static_cast<int>(_prevNearbyRobots.size());
}

std::string CoopFixed2OpportunityObj::inspect()
{
    std::stringstream out;
    out << "I had agents previously present: ";
    for (auto prevAgent: _prevNearbyRobots)
    {
        out << prevAgent << ", ";
    }
    out << ".\n";
    out << "I have these agents present:";
    for (auto prevAgent: _curNearbyRobots)
    {
        out << prevAgent << ", ";
    }
    out << "\n";
    return out.str();
}
