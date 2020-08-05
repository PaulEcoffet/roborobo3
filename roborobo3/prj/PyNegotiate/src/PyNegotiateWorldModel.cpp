/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <RoboroboMain/main.h>
#include "PyNegotiate/include/PyNegotiateWorldModel.h"
#include <boost/algorithm/clamp.hpp>
#include <PyNegotiate/include/PyNegotiateSharedData.h>
#include <PyNegotiate/include/PyNegotiateWorldModel.h>
#include <core/World/World.h>
#include <boost/algorithm/clamp.hpp>


using boost::algorithm::clamp;


PyNegotiateWorldModel::PyNegotiateWorldModel()
        : RobotWorldModel(),
          onOpportunity(false),
          selfA(0.5),
          opp(nullptr),
          fake(false),
          fakeCoef(1),
          teleport(false),
          punishment(0),
          spite(0)
{
    setNewSelfA();
}

void PyNegotiateWorldModel::setNewSelfA()
{

    // set the selfA, it makes a non-gaussian ESS distribution though TODO see if skewed is okay
    selfA = std::max(randgaussian() * PyNegotiateSharedData::stdA + PyNegotiateSharedData::meanA, 0.01);
}


void PyNegotiateWorldModel::reset()
{
    onOpportunity = false;
    teleport = false;
    toBeTeleported = false;
    opp = nullptr;
}


bool PyNegotiateWorldModel::isPlaying() const
{
    return onOpportunity && (arrival <= 2 || !PyNegotiateSharedData::fixRobotNb) &&
           (!PyNegotiateSharedData::atLeastTwo || nbOnOpp >= 2);
}

double PyNegotiateWorldModel::getCoop(bool trueValue) const
{
    double coop = _cooperationLevel;
    if (!trueValue)
    {
        coop = clamp(coop + fakeCoef, 0, PyNegotiateSharedData::maxCoop);
    }
    return coop;
}

