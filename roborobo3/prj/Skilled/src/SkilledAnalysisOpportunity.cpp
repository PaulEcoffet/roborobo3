/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#include <Skilled/include/SkilledSharedData.h>
#include <Skilled/include/SkilledAnalysisWorldObserver.h>
#include <core/WorldModels/RobotWorldModel.h>
#include <Skilled/include/SkilledAnalysisOpportunity.h>

#include "Skilled/include/SkilledAnalysisOpportunity.h"


#include "RoboroboMain/roborobo.h"
#include "Skilled/include/SkilledAnalysisOpportunity.h"

SkilledAnalysisOpportunity::SkilledAnalysisOpportunity(int __id) : SkilledOpportunity(__id)
{
    setType(10);
}


void SkilledAnalysisOpportunity::step()
{
    updateColor();
    for (auto *fakerobot : fakerobots)
    {
        RobotWorldModel *wm = fakerobot->getWorldModel();
        wm->_desiredTranslationalValue = 0;
        wm->_cooperationLevel = getCoop(); // TODO WRONG COOP
        wm->_desiredRotationalVelocity = 0;
    }
}

void SkilledAnalysisOpportunity::setCoopValue(double coop)
{
    m_coop = coop;
    updateColor();
}

double SkilledAnalysisOpportunity::getCoop() const
{
    return m_coop;
}

void SkilledAnalysisOpportunity::setNbFakeRobots(int nbrobots)
{
    nbFakeRobots = nbrobots;

}

int SkilledAnalysisOpportunity::getNbFakeRobots()
{
    return nbFakeRobots;
}

void SkilledAnalysisOpportunity::placeFakeRobot()
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
        wm->_cooperationLevel = getCoop(); // TODO wrong coop
        wm->_desiredRotationalVelocity = 0;
        wm->setAlive(false);
        isWalked(wm->getId() + gRobotIndexStartOffset);
        i++;
    }

}

void SkilledAnalysisOpportunity::updateColor()
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

