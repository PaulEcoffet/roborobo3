/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#include <NegociateGym/include/NegociateGymSharedData.h>
#include <NegociateGym/include/NegociateGymAnalysisWorldObserver.h>
#include <core/WorldModels/RobotWorldModel.h>
#include <NegociateGym/include/NegociateGymAnalysisOpportunity.h>

#include "NegociateGym/include/NegociateGymAnalysisOpportunity.h"


#include "RoboroboMain/roborobo.h"
#include "NegociateGym/include/NegociateGymAnalysisOpportunity.h"

NegociateGymAnalysisOpportunity::NegociateGymAnalysisOpportunity(int __id) : NegociateGymOpportunity(__id)
{
    setType(10);
}


void NegociateGymAnalysisOpportunity::step()
{
    updateColor();
    for (auto *fakerobot : fakerobots)
    {
        RobotWorldModel *wm = fakerobot->getWorldModel();
        wm->_desiredTranslationalValue = 0;
        wm->_cooperationLevel = getCoop();
        wm->_desiredRotationalVelocity = 0;
    }
}

void NegociateGymAnalysisOpportunity::setCoopValue(double coop)
{
    m_coop = coop;
    updateColor();
}

double NegociateGymAnalysisOpportunity::getCoop() const
{
    return m_coop;
}

void NegociateGymAnalysisOpportunity::setNbFakeRobots(int nbrobots)
{
    nbFakeRobots = nbrobots;

}

int NegociateGymAnalysisOpportunity::getNbFakeRobots()
{
    return nbFakeRobots;
}

void NegociateGymAnalysisOpportunity::placeFakeRobot()
{
    if (nbFakeRobots < 1 || fakerobots.empty())
    {
        return;
    }

    assert (nbFakeRobots == fakerobots.size());

    const int cx = getXCenterPixel();
    const int cy = getYCenterPixel();
    const double rotphase = fmod((double) _id / 50, 1);
    int i = 1;
    for (auto *fakerobot : fakerobots)
    {
        const double pos = (double) i / (fakerobots.size() + 1);
        const double minrot = 1. / 6;
        const double maxrot = 5. / 6;
        const double linrot = rotphase + minrot + (maxrot - minrot) * pos;
        const double rot = linrot * 2 * M_PI;
        const int objradius = gPhysicalObjectDefaultRadius;
        const int robh = gRobotHeight;
        const int robw = gRobotWidth;
        int newrobcx = static_cast<int>(round(cx + (objradius + robw / 2) * cos(rot)));
        int newrobcy = static_cast<int>(round(cy + (objradius + robh / 2) * sin(rot)));
        int xpad = 0, ypad = 0;
        fakerobot->unregisterRobot();
        fakerobot->setCoordReal(newrobcx + xpad, newrobcy + ypad);
        fakerobot->setCoord(newrobcx + xpad, newrobcy + ypad);
        fakerobot->registerRobot();
        RobotWorldModel *wm = fakerobot->getWorldModel();
        wm->_desiredTranslationalValue = 0;
        wm->_agentAbsoluteOrientation = 0;
        wm->_cooperationLevel = getCoop();
        wm->_desiredRotationalVelocity = 0;
        wm->setAlive(false);
        isWalked(wm->getId() + gRobotIndexStartOffset);
        i++;
    }

}

void NegociateGymAnalysisOpportunity::updateColor()
{
    if (m_coop * nbFakeRobots < 5)
    {
        _displayColorRed = 189;
        _displayColorGreen = 131;
        _displayColorBlue = 126;
    }
    else if (m_coop * nbFakeRobots < 10)
    {
        _displayColorRed = 198;
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

