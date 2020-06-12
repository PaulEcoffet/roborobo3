//
// Created by paul on 30/10/17.
//


#include <WorldModels/RobotWorldModel.h>
#include <pyFastWanderer/include/pyFastWandererWorldObserver.h>
#include "RoboroboMain/main.h"
#include "pyFastWanderer/include/pyFastWandererSharedData.h"
#include <boost/algorithm/string.hpp>

pyFastWandererWorldObserver::pyFastWandererWorldObserver(World *__world) : WorldObserver(__world)
{

    m_curEvalutionIteration = 0;
    _generationCount = 0;

    pyFastWandererSharedData::initSharedData();

    // Log files

    std::string fitnessLogFilename = gLogDirectoryname + "/fitnesslog.txt";
    m_fitnessLogManager = new LogManager(fitnessLogFilename);
    m_fitnessLogManager->write("gen\tpop\tminfit\tq1fit\tmedfit\tq3fit\tmaxfit\tmeanfit\tvarfit\n");

    gMaxIt = -1;
}


pyFastWandererWorldObserver::~pyFastWandererWorldObserver()
{
    delete m_fitnessLogManager;
}

void pyFastWandererWorldObserver::reset()
{
    resetEnvironment();
    clearRobotFitnesses();
}

void pyFastWandererWorldObserver::stepPre()
{

    resetEnvironment();
    clearRobotFitnesses();
    m_curEvalutionIteration++;
}


void pyFastWandererWorldObserver::stepPost()
{
    for (int i = 0; i < gWorld->getNbOfRobots(); i++)
    {
        RobotWorldModel *wm = gWorld->getRobot(i)->getWorldModel();
        double speed = (wm->_actualTranslationalValue / gMaxTranslationalSpeed);
        double rotspeed = (wm->_actualRotationalVelocity / gMaxRotationalSpeed);
        double closestObjDistance = 1;
        for (int j = 0; j < wm->_cameraSensorsNb; j++)
        {
            double cur_dist = wm->getDistanceValueFromCameraSensor(j) / wm->getCameraSensorMaximumDistanceValue(j);
            if (cur_dist < closestObjDistance)
            {
                closestObjDistance = cur_dist;
            }
        }
        wm->_fitnessValue += speed * (1 - sqrt(fabs(rotspeed))) * closestObjDistance;
    }
}


void pyFastWandererWorldObserver::resetEnvironment()
{
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->unregisterRobot();
    }

    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->reset();
        robot->registerRobot();
    }
}


void pyFastWandererWorldObserver::clearRobotFitnesses()
{
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        auto* ctl = dynamic_cast<pyFastWandererController*>(gWorld->getRobot(iRobot)->getController());
        ctl->resetFitness();
    }
}