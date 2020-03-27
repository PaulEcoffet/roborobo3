//
// Created by paul on 31/10/17.
//

#include <Skilled/include/SkilledWorldObserver.h>
#include <Skilled/include/SkilledSharedData.h>
#include "RoboroboMain/roborobo.h"
#include "World/World.h"
#include "Skilled/include/SkilledOpportunity.h"

SkilledOpportunity::SkilledOpportunity(int __id) : RoundObject(__id) {
    setType(9);
    curA = 5;
    lifeid = __id;
    curInv = 0;
    ifNewPartInv = 0;
    if (SkilledSharedData::oppDecay > 0) {
        lifeExpectancy = 1. / SkilledSharedData::oppDecay;
    } else {
        lifeExpectancy = 0;
    }
}


void SkilledOpportunity::removeRobot(int id) {
    // REMOVE THE ROBOT
    nearbyMap.erase(id);

    // Keep track of what's going on
    int nbonopp = countCurrentRobots();
    int nbonoppifnewpart = nbonopp + 1;

    if (nbonopp > 0) {
        curInv = sumCoop(nbonopp);
        ifNewPartInv = sumCoop(std::min(nbonoppifnewpart, gInitialNumberOfRobots));
    } else {
        ifNewPartInv = 0;
        curInv = 0;
    }
    assert(curInv >= 0);
    assert(ifNewPartInv >= 0);
}

void SkilledOpportunity::isWalked(const int id) {
    const int rid = id - gRobotIndexStartOffset;
    auto *wm = dynamic_cast<SkilledWorldModel *>(gWorld->getRobot(rid)->getWorldModel());
    if (!isRobotOnOpp(rid)) {
        nearbyMap.emplace(rid, wm);
        int nbonopp = countCurrentRobots();
        int nbpartnerperrobot = nbonopp - 1;
        int nbonoppifnewpart = nbonopp + 1;
        curInv = sumCoop(nbonopp);

        /*
        std::cout << "*********\n";
        std::cout << "oppid:" << _id << "\n";
        std::cout << "cur robs: " << countCurrentRobots() << "\n";
        std::cout << curInv << " = " << ifNewPartInv << " + " << wm->getCoop(nbpartnerperrobot) << "\n";
        std::cout << curInv - (ifNewPartInv + wm->getCoop(nbpartnerperrobot)) << std::endl;
         */
        assert(fabs(curInv - (ifNewPartInv + wm->getCoop(nbpartnerperrobot))) < 1e-5);

        ifNewPartInv = sumCoop(std::min(nbonoppifnewpart, gInitialNumberOfRobots));
    }
}

int SkilledOpportunity::countCurrentRobots() {
    return static_cast<int>(nearbyMap.size());
}

bool SkilledOpportunity::isRobotOnOpp(const int id) {
    return nearbyMap.count(id) > 0;
}

void SkilledOpportunity::step() {
    if (random01() < lifeExpectancy) {
        kill();
    }
    updateColor();
    RoundObject::step();
}

void SkilledOpportunity::kill() {
    auto *wobs = dynamic_cast<SkilledWorldObserver *>(gWorld->getWorldObserver());
    reset();
    wobs->addObjectToTeleport(_id);
}

void SkilledOpportunity::updateColor() {
    if (countCurrentRobots() == 0) {
        _displayColorRed = 152;
        _displayColorGreen = 200;
        _displayColorBlue = 158;
    } else {
        if (curInv < curA * 0.8 - 0.5) /* under xESS = a/n per robot, so for sum over rob xESS = A */
        {
            _displayColorRed = 189;
            _displayColorGreen = 131;
            _displayColorBlue = 126;
        } else if (curInv < curA * 1.2 + 1) /* basically playing ESS */
        {
            _displayColorRed = 198;
            _displayColorGreen = 186;
            _displayColorBlue = 58;
        } else /* Playing above ESS, closer to SO */
        {
            _displayColorRed = 44;
            _displayColorGreen = 83;
            _displayColorBlue = 120;
        }
    }
}

std::string SkilledOpportunity::inspect(std::string prefix) {
    std::stringstream out;
    out << prefix << "Last inv: " << curInv << ".\n";
    out << prefix << "I have " << countCurrentRobots() << " robots on me: ";
    for (auto elem : nearbyMap) {
        out << elem.first << ",";
    }
    out << "\n";
    return out.str();
}

double SkilledOpportunity::getCurInv() const {
    return curInv;
}

double SkilledOpportunity::getIfNewPartInv() const {
    return ifNewPartInv;
}

double SkilledOpportunity::sumCoop(int nbonopp) {
    if (nbonopp == 0) return 0;

    int nbpart = nbonopp - 1;
    assert(nbpart >= 0 && nbpart < gNbOfRobots);
    double tot = 0;
    for (auto elem : nearbyMap) {
        tot += elem.second->getCoop(nbpart);
    }
    assert(tot >= 0);
    return tot;
}

void SkilledOpportunity::reset() {
    for (auto rob: nearbyMap) {
        dynamic_cast<SkilledWorldModel *>(gWorld->getRobot(rob.first)->getWorldModel())->opp = nullptr;
    }
    nearbyMap.clear();
    curInv = 0;
    ifNewPartInv = 0;

}
