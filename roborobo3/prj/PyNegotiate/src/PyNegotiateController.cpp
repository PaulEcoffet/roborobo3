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
        auto inputs_py = m_wm->getObservations();
        out << prefix << "inputs:\n";
        int i = 0;
        for (auto elem : inputs_py["obs"])
        {
            out << prefix << "\t" << inputnames[i] << ":" << elem.str().cast<std::string>() << "\n";
            i++;
        }
    }
    verbose = (verbose + 1) % 2;
    return out.str();
}
