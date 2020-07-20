/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <RoboroboMain/main.h>
#include "Skilled/include/SkilledWorldModel.h"
#include <boost/algorithm/clamp.hpp>
#include <Skilled/include/SkilledSharedData.h>
#include <Skilled/include/SkilledWorldModel.h>
#include <core/World/World.h>


SkilledWorldModel::SkilledWorldModel()
        : RobotWorldModel(),
          arrival(0),
          onOpportunity(false),
          selfA(0.5),
          opp(nullptr),
          fake(false),
          ingame(false),
          fakeCoef(1),
          teleport(false)
{
    setNewSelfA();
}

void SkilledWorldModel::setNewSelfA()
{

    selfA = std::max(randgaussian() * SkilledSharedData::stdA + SkilledSharedData::meanA, 0.01);
}


void SkilledWorldModel::reset()
{
    onOpportunity = false;
    nbOnOpp = 0;
    arrival = 0;
    teleport = false;
    opp = nullptr;
}

bool SkilledWorldModel::isPlaying()
{
    return opp != nullptr;
}


double SkilledWorldModel::getCoop(int nbpart, bool truecoop)
{
    assert(nbpart >= 0 && nbpart < gInitialNumberOfRobots);
    double coop;
    if (nbpart == 0)
    {
        coop = coopalone;
    }
    else
    {
        coop = cooppartner;
    }
    if (!(truecoop || SkilledSharedData::independantCoop))
    {
        if (!SkilledSharedData::additiveVar)
            coop *= fakeCoef;
        else
            coop += fakeCoef - 1;

    }
    return boost::algorithm::clamp(coop, 0, SkilledSharedData::maxCoop);
}

void SkilledWorldModel::setCoopAlone(double val)
{
    assert(val >= 0 && val <= SkilledSharedData::maxCoop);
    coopalone = val;
}

void SkilledWorldModel::setCoopPartners(double val)
{
    assert(val >= 0 && val <= SkilledSharedData::maxCoop);
    cooppartner = val;
}

void SkilledWorldModel::setCoops(double _coopalone, double _cooppartner)
{
    setCoopAlone(_coopalone);
    setCoopPartners(_cooppartner);
}

double SkilledWorldModel::getSkill() const
{
    return skill;
}

void SkilledWorldModel::setSkill(double skill)
{
    assert(0 <= skill && skill <= 1);
    SkilledWorldModel::skill = skill;
}
