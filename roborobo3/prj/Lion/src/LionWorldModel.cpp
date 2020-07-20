/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <RoboroboMain/main.h>
#include "Lion/include/LionWorldModel.h"
#include <boost/algorithm/clamp.hpp>
#include <Lion/include/LionSharedData.h>
#include <Lion/include/LionWorldModel.h>
#include <core/World/World.h>


LionWorldModel::LionWorldModel()
        : RobotWorldModel(),
          onOpportunity(false),
          selfA(0.5),
          opp(nullptr),
          fake(false),
          ingame(false),
          fakeCoef(1),
          teleport(false),
          coopCache(gInitialNumberOfRobots)
{
    setNewSelfA();
}

void LionWorldModel::setNewSelfA()
{

    selfA = std::max(randgaussian() * LionSharedData::stdA + LionSharedData::meanA, 0.01);
}


void LionWorldModel::reset()
{
    onOpportunity = false;
    nbOnOpp = 0;
    arrival = 0;
    teleport = false;
    opp = nullptr;
}

bool LionWorldModel::isPlaying()
{
    return opp != nullptr;
}


double LionWorldModel::getCoop(int nbpart, bool truecoop)
{
    assert(nbpart >= 0 && nbpart < gInitialNumberOfRobots);
    assert(coopCache[nbpart] >= 0 && coopCache[nbpart] <= LionSharedData::maxCoop);
    if (truecoop || LionSharedData::independantCoop)
        return coopCache[nbpart];
    else
    {
        if (!LionSharedData::additiveVar)
            return coopCache[nbpart] * fakeCoef;
        else
            return boost::algorithm::clamp(coopCache[nbpart] + fakeCoef - 1, 0, LionSharedData::maxCoop);
    }
}

void LionWorldModel::setCoop(int nbpart, double val)
{
    assert(nbpart >= 0 && nbpart < gInitialNumberOfRobots);
    assert(val >= 0 && val <= LionSharedData::maxCoop);
    coopCache[nbpart] = val;
}