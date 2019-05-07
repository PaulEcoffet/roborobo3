//
// Created by paul on 31/10/17.
//

#include <Lion/include/LionWorldObserver.h>
#include <Lion/include/LionSharedData.h>
#include "RoboroboMain/roborobo.h"
#include "World/World.h"
#include "Lion/include/LionOpportunity.h"

LionOpportunity::LionOpportunity(int __id) : RoundObject(__id)
{
    setType(9);
    curA = 5;
    lifeid = __id;
    curInv = 0;
    ifNewPartInv = 0;
    if (LionSharedData::oppDecay > 0)
    {
        lifeExpectancy = 1. / LionSharedData::oppDecay;
    }
    else
    {
        lifeExpectancy = 0;
    }
}


void LionOpportunity::removeRobot(int id)
{
    nearbyMap.erase(id);
    curInv = sumCoop(countCurrentRobots());
    ifNewPartInv = sumCoop(std::min(countCurrentRobots() + 1, gInitialNumberOfRobots - 1));
    assert(curInv >= 0);
    assert(ifNewPartInv >= 0);
}

void LionOpportunity::isWalked(const int id)
{
    const int rid = id - gRobotIndexStartOffset;
    auto* wm = dynamic_cast<LionWorldModel*>(gWorld->getRobot(rid)->getWorldModel());
    nearbyMap.emplace(rid, wm);
    curInv = sumCoop(countCurrentRobots());

    assert(curInv - (ifNewPartInv + wm->getCoop(countCurrentRobots() - 1) < 1e10));

    ifNewPartInv = sumCoop(std::min(countCurrentRobots(), gInitialNumberOfRobots + 1));

}

int LionOpportunity::countCurrentRobots()
{
    return static_cast<int>(nearbyMap.size());
}


void LionOpportunity::step()
{
    if (random() < lifeExpectancy)
    {
        kill();
    }
    updateColor();
    RoundObject::step();
}

void LionOpportunity::kill()
{
    auto *wobs = dynamic_cast<LionWorldObserver *>(gWorld->getWorldObserver());
    reset();
    wobs->addObjectToTeleport(_id);
}

void LionOpportunity::updateColor()
{
    if (countCurrentRobots() == 0)
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

std::string LionOpportunity::inspect(std::string prefix)
{
    std::stringstream out;
    out << prefix << "Last inv: " << curInv << ".\n";
    out << prefix << "I have " << countCurrentRobots() << " robots on me: ";
    for (auto elem : nearbyMap)
    {
        out << elem.first << ",";
    }
    out << "\n";
    return out.str();
}

double LionOpportunity::getCurInv() const
{
    return curInv;
}

double LionOpportunity::getIfNewPartInv() const
{
    return ifNewPartInv;
}

double LionOpportunity::sumCoop(int nbonopp)
{
    if (nbonopp == 0) return 0;

    int nbpart = nbonopp - 1;
    assert(nbpart >= 0 && nbpart < gNbOfRobots);
    double tot = 0;
    for(auto elem : nearbyMap)
    {
        tot += elem.second->getCoop(nbpart);
    }
    assert(tot >= 0);
    return tot;
}

void LionOpportunity::reset()
{
    nearbyMap.clear();
    curInv = 0;
    ifNewPartInv = 0;

}
