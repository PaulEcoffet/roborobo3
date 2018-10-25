/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#include <CoopFixed2/include/CoopFixed2SharedData.h>
#include <CoopFixed2/include/CoopFixed2AnalysisWorldObserver.h>
#include <core/WorldModels/RobotWorldModel.h>
#include <CoopFixed2/include/CoopFixed2AnalysisOpportunity.h>

#include "CoopFixed2/include/CoopFixed2AnalysisOpportunity.h"


#include "RoboroboMain/roborobo.h"
#include "CoopFixed2/include/CoopFixed2AnalysisOpportunity.h"

CoopFixed2AnalysisOpportunity::CoopFixed2AnalysisOpportunity(int __id) : CoopFixed2Opportunity(__id)
{
    setType(10);
    lifeExpectancy = 1. / CoopFixed2SharedData::oppDecay;
}


void CoopFixed2AnalysisOpportunity::step()
{
    bool killed = random() < lifeExpectancy;
    if (killed)
    {
        auto *wobs = dynamic_cast<CoopFixed2AnalysisWorldObserver *>(gWorld->getWorldObserver());
        nearbyRobotIndexes.clear();
        newNearbyRobotIndexes.clear();
        wobs->addObjectToTeleport(_id);
    }
    updateColor();
    if (fakerobot != nullptr)
    {
        RobotWorldModel *wm = fakerobot->getWorldModel();
        wm->_desiredTranslationalValue = 0;
        wm->_cooperationLevel = getCoop();
        wm->_desiredRotationalVelocity = 0;
    }
}

void CoopFixed2AnalysisOpportunity::setCoopValue(double coop)
{
    m_coop = coop;
    updateColor();
}

double CoopFixed2AnalysisOpportunity::getCoop() const
{
    return m_coop;
}

void CoopFixed2AnalysisOpportunity::setNbFakeRobots(int nbrobots)
{
    nbFakeRobots = nbrobots;

}

int CoopFixed2AnalysisOpportunity::getNbFakeRobots()
{
    return nbFakeRobots;
}

void CoopFixed2AnalysisOpportunity::placeFakeRobot()
{
    if (nbFakeRobots < 1 || fakerobot == nullptr)
    {
        return;
    }

    const int cx = getXCenterPixel();
    const int cy = getYCenterPixel();
    const double rot = fmod((double) lifeid / gNbOfPhysicalObjects * 2 * M_PI, 2.0 * M_PI);
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
}

void CoopFixed2AnalysisOpportunity::updateColor()
{
    if (m_coop < 3) /* under xESS = a/n per robot, so for sum over rob xESS = A */
    {
        _displayColorRed = 189;
        _displayColorGreen = 131;
        _displayColorBlue = 126;
    }
    else if (m_coop < 6) /* basically playing ESS */
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

