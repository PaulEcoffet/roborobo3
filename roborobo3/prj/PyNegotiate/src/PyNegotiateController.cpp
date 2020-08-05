//
// Created by paul on 27/10/17.
//

#include <contrib/neuralnetworks/EigenMLP.h>
#include "neuralnetworks/Perceptron.h"
#include "Utilities/Misc.h"
#include "WorldModels/RobotWorldModel.h"
#include "Agents/Agent.h"
#include "World/World.h"
#include "RoboroboMain/main.h"
#include <set>
#include <PyNegotiate/include/PyNegotiateWorldModel.h>
#include "neuralnetworks/Elman.h"
#include "PyNegotiate/include/PyNegotiateController.h"
#include "PyNegotiate/include/PyNegotiateSharedData.h"

enum
{
    MLP_ID = 0,
    PERCEPTRON_ID = 1,
    ELMAN_ID = 2
};

std::vector<std::string> PyNegotiateController::inputnames;

PyNegotiateController::PyNegotiateController(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<PyNegotiateWorldModel *>(wm);
    fill_names = true;

    fillNames();
    resetFitness();
}

void PyNegotiateController::reset()
{
}

void PyNegotiateController::step()
{
    verbose = 0;
    if (not m_wm->seeking)
    {
        if (PyNegotiateSharedData::wander)
        {
            wander_behavior();
        }
    }
}

void PyNegotiateController::wander_behavior() const
{
    const double sensorlength = 30; // pixels
    double meandistfront = (
                                   m_wm->getCameraSensorValue(2, SENSOR_DISTANCEVALUE) +
                                   m_wm->getCameraSensorValue(3, SENSOR_DISTANCEVALUE)
                           ) / 2;
    if (meandistfront > sensorlength)
    {
        meandistfront = sensorlength;
    }
    m_wm->_desiredTranslationalValue = +2 - 2 * ((sensorlength - meandistfront) / sensorlength);

    if (m_wm->getCameraSensorValue(0, SENSOR_DISTANCEVALUE) +
        m_wm->getCameraSensorValue(1, SENSOR_DISTANCEVALUE) +
        m_wm->getCameraSensorValue(2, SENSOR_DISTANCEVALUE) <
        m_wm->getCameraSensorValue(3, SENSOR_DISTANCEVALUE) +
        m_wm->getCameraSensorValue(4, SENSOR_DISTANCEVALUE) +
        m_wm->getCameraSensorValue(5, SENSOR_DISTANCEVALUE))
    {
        m_wm->_desiredRotationalVelocity = +10;
    }
    else if (m_wm->getCameraSensorValue(3, SENSOR_DISTANCEVALUE) +
             m_wm->getCameraSensorValue(4, SENSOR_DISTANCEVALUE) +
             m_wm->getCameraSensorValue(5, SENSOR_DISTANCEVALUE) < 3 * gSensorRange)
    {
        m_wm->_desiredRotationalVelocity = -10;
    }
    else if (m_wm->_desiredRotationalVelocity > 0)
    {
        m_wm->_desiredRotationalVelocity--;
    }
    else if (m_wm->_desiredRotationalVelocity < 0)
    {
        m_wm->_desiredRotationalVelocity++;
    }
    else
    {
        m_wm->_desiredRotationalVelocity = 0.01 - (double) (randint() % 10) / 10. * 0.02;
    }
}


PyNegotiateController::~PyNegotiateController() = default;

std::vector<double> PyNegotiateController::getInputs()
{
    std::vector<double> inputs;
    inputs = getCameraInputs();

    const std::vector<double> game_inputs(getGameInputs());
    inputs.insert(inputs.end(), game_inputs.begin(), game_inputs.end());
    fill_names = false;
    assert(inputnames.size() == inputs.size());
    return inputs;
}

std::vector<double> PyNegotiateController::getCameraInputs() const
{
    size_t i = 0;
    std::vector<double> inputs(getNbCameraInputs(), 0);
    const int WALL_ID = 0;
    /*
     * Camera inputs
     */
    for (int j = 0; j < m_wm->_cameraSensorsNb; j++)
    {
        bool isOpportunity = false;
        double nbOnOpp = 0;
        auto entityId = static_cast<int>(m_wm->getObjectIdFromCameraSensor(j));

        if (entityId >= gPhysicalObjectIndexStartOffset &&
            entityId < gPhysicalObjectIndexStartOffset + gNbOfPhysicalObjects) // is an Object
        {
            auto *opportunity = dynamic_cast<PyNegotiateOpportunity *>(
                    gPhysicalObjects[entityId - gPhysicalObjectIndexStartOffset]);
            isOpportunity = true;
            if (opportunity == m_wm->opp)
            {
                nbOnOpp = m_wm->nbOnOpp;
            }
            else
            {
                nbOnOpp = opportunity->getNbNearbyRobots();
            }
        }
        double dist = m_wm->getDistanceValueFromCameraSensor(j) / m_wm->getCameraSensorMaximumDistanceValue(j);
        inputs[i++] = (Agent::isInstanceOf(entityId)) ? dist : 1;
        inputs[i++] = (entityId == WALL_ID) ? dist : 1;
        inputs[i++] = (isOpportunity) ? dist : 1;
        inputs[i++] = nbOnOpp;
    }

    return inputs;
}


void PyNegotiateController::fillNames()
{
    if (inputnames.empty())
    {
        for (int j = 0; j < m_wm->_cameraSensorsNb; j++)
        {
            inputnames.emplace_back("dist robot");
            inputnames.emplace_back("dist wall");
            inputnames.emplace_back("dist obj");
            inputnames.emplace_back("nb on obj");
        }
        if (PyNegotiateSharedData::totalInvAsInput)
        {
            inputnames.emplace_back("mean total inv");

        }
        if (PyNegotiateSharedData::ownInvAsInput)
        {
            inputnames.emplace_back("mean own inv");
        }

        fill_names = false;
    }
}

std::vector<double> PyNegotiateController::getGameInputs() const
{
    std::vector<double> inputs(getNbGameInputs(), 0);
    size_t i = 0;
    if (PyNegotiateSharedData::totalInvAsInput)
    {
        inputs[i++] =  /* TODO get total invest */ 0 /* end TODO */ / PyNegotiateSharedData::maxCoop;
    }
    if (PyNegotiateSharedData::ownInvAsInput)
    {
        inputs[i++] = m_wm->getCoop() / PyNegotiateSharedData::maxCoop;
    }
    return inputs;
}

unsigned int PyNegotiateController::getNbInputs() const
{
    return getNbCameraInputs() + getNbGameInputs();
}

unsigned int PyNegotiateController::getNbGameInputs() const
{
    return 2;
}

unsigned int PyNegotiateController::getNbCameraInputs() const
{
    const auto nbCameraInputs = static_cast<const unsigned int>(
            m_wm->_cameraSensorsNb * (4)); // distWall + distRobot + distObj + nbRob
    return nbCameraInputs;
}


double PyNegotiateController::getFitness() const
{
    return m_wm->_fitnessValue;
}


void PyNegotiateController::resetFitness()
{
    updateFitness(0);
}

void PyNegotiateController::updateFitness(double newFitness)
{
    if (newFitness < 0)
    {
        updateFitness(0);
        return;
    }
    m_wm->_fitnessValue = newFitness;
}

void PyNegotiateController::increaseFitness(double delta)
{
    updateFitness(m_wm->_fitnessValue + delta);
}

std::string PyNegotiateController::inspect(std::string prefix)
{
    std::stringstream out;
    if (verbose == 0)
    {
        out << prefix << "last coop: " << m_wm->getCoop() << " (true val: " << m_wm->getCoop(true) << ")\n";
        out << prefix << "Actual fitness: " << getFitness() << "\n";
    }
    if (verbose == 1)
    {
        auto inputs = wm->getInputs();
        out << prefix << "inputs:\n";
        for (size_t i = 0; i < inputs.size(); i++)
        {
            out << prefix << "\t" << inputnames[i] << ":" << inputs[i] << "\n";
        }
    }
    verbose = (verbose + 1) % 2;
    return out.str();
}

unsigned int PyNegotiateController::getNbOutputs() const
{
    return 1
           + (unsigned int) !PyNegotiateSharedData::tpToNewObj // Motor commands
           + (unsigned int) !PyNegotiateSharedData::fixCoop  // Cooperation value
            ;
}
