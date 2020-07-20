/*
 * MovingObject2Max.cpp
 *
 *  Created on: 9 oct. 2017
 *      Author: Paul Ecoffet
 */

#include "CoopOpportunity2Max/include/CoopOpportunity2MaxMovingObject2Max.h"
#include "CoopOpportunity2Max/include/CoopOpportunity2MaxSharedData.h"
#include "RoboroboMain/roborobo.h"


CoopOpportunity2MaxMovingObject2Max::CoopOpportunity2MaxMovingObject2Max(int __id) : MovingObject(__id)
{
    setType(6);

}

CoopOpportunity2MaxMovingObject2Max::~CoopOpportunity2MaxMovingObject2Max()
{
}

void CoopOpportunity2MaxMovingObject2Max::step()
{
    _prevNearbyRobots.clear();
    _prevNearbyRobots.insert(_nearbyRobots.begin(), _nearbyRobots.end());
    MovingObject::step();
}

void CoopOpportunity2MaxMovingObject2Max::isPushed(int __idAgent, std::tuple<double, double> __speed)
{
    if (_nearbyRobots.size() == 2 && _nearbyRobots.count(__idAgent - gRobotIndexStartOffset) == 0)
    {
        if (gVerbose)
        {
            std::cout << _id << ": Already too much robots!, no room for " << __idAgent - gRobotIndexStartOffset
                      << "\n";
            std::cout << "current robots: ";
        }
        if (gVerbose)
        {
            for (auto prevrob : _nearbyRobots)
            {
                std::cout << prevrob << " ";
            }
            std::cout << "prev robots: ";
            for (auto prevrob : _prevNearbyRobots)
            {
                std::cout << prevrob << " ";
            }
            std::cout << "\n";
        }
        if (_prevNearbyRobots.count(__idAgent - gRobotIndexStartOffset) > 0) // This robot was here at the previous step
        {
            if (gVerbose)
            {
                std::cout << __idAgent - gRobotIndexStartOffset << " was already there last turn\n";
            }
            // We make room for this pre-existing robot.
            for (auto i = _nearbyRobots.begin(); i != _nearbyRobots.end(); i++)
            {
                auto nearby_robot = *i;
                if (_prevNearbyRobots.count(nearby_robot) == 0)
                {
                    if (gVerbose)
                    {
                        std::cout << nearby_robot << " was not here before, kicking him!\n";
                    }
                    _nearbyRobots.erase(i);
                    _impulses.erase(nearby_robot);
                    break;
                }
            }
        }
        else // If the robot was not already here and the moving object is full, we do nothing
        {
            return;
        }
    }
    if (_impulses.count(__idAgent) == 0)
    {
        _impulses.insert(std::pair<int, std::tuple<double, double>>(__idAgent, __speed));
        _nearbyRobots.insert(__idAgent - gRobotIndexStartOffset);
    }

    assert(_nearbyRobots.size() <= 2);
}
