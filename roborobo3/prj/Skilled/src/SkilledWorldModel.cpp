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
          onOpportunity(false),
          selfA(0.5),
          opp(nullptr),
          fake(false),
          ingame(false),
          fakeCoef(1),
          teleport(false),
          coopCache(gInitialNumberOfRobots) {
    setNewSelfA();
}

void SkilledWorldModel::setNewSelfA() {

    selfA = std::max(randgaussian() * SkilledSharedData::stdA + SkilledSharedData::meanA, 0.01);
}


void SkilledWorldModel::reset() {
    onOpportunity = false;
    nbOnOpp = 0;
    arrival = 0;
    teleport = false;
    opp = nullptr;
}

bool SkilledWorldModel::isPlaying() {
    return opp != nullptr;
}


double SkilledWorldModel::getCoop(int nbpart, bool truecoop) {
    assert(nbpart >= 0 && nbpart < gInitialNumberOfRobots);
    assert(coopCache[nbpart] >= 0 && coopCache[nbpart] <= SkilledSharedData::maxCoop);
    if (truecoop || SkilledSharedData::independantCoop)
        return coopCache[nbpart];
    else {
        if (!SkilledSharedData::additiveVar)
            return coopCache[nbpart] * fakeCoef;
        else
            return boost::algorithm::clamp(coopCache[nbpart] + fakeCoef - 1, 0, SkilledSharedData::maxCoop);
    }
}

void SkilledWorldModel::setCoop(int nbpart, double val) {
    assert(nbpart >= 0 && nbpart < gInitialNumberOfRobots);
    assert(val >= 0 && val <= SkilledSharedData::maxCoop);
    coopCache[nbpart] = val;
}