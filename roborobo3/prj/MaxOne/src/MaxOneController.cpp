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
#include <MaxOne/include/MaxOneWorldModel.h>
#include "neuralnetworks/Elman.h"
#include "MaxOne/include/MaxOneController.h"

enum
{
    MLP_ID = 0,
    PERCEPTRON_ID = 1,
    ELMAN_ID = 2
};


MaxOneController::MaxOneController(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<MaxOneWorldModel *>(wm);

    weights.resize(10);

    resetFitness();
}

void MaxOneController::reset()
{
}

void MaxOneController::step()
{

}



MaxOneController::~MaxOneController()
{
}

void MaxOneController::loadNewGenome(const std::vector<double> &newGenome)
{
    weights = newGenome;
}

double MaxOneController::getFitness() const
{
    return m_wm->_fitnessValue;
}


void MaxOneController::resetFitness()
{
    updateFitness(0);
}

void MaxOneController::updateFitness(double newFitness)
{
    if (newFitness < 0)
    {
        updateFitness(0);
        return;
    }
    m_wm->_fitnessValue = newFitness;
}

void MaxOneController::increaseFitness(double delta)
{
    updateFitness(m_wm->_fitnessValue + delta);
}

std::string MaxOneController::inspect(std::string prefix) {
    std::stringstream out;

    for (auto &output : weights)
    {
        out << prefix << "\t" << output << "\n"; // TODO Crash without reason
    }

    return out.str();
}

std::vector<double> MaxOneController::getWeights() const
{
    return weights;
}
