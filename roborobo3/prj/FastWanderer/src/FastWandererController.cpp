//
// Created by paul on 27/10/17.
//

#include "neuralnetworks/Perceptron.h"
#include "Utilities/Misc.h"
#include "WorldModels/RobotWorldModel.h"
#include "Agents/Agent.h"
#include "RoboroboMain/main.h"
#include <set>
#include "neuralnetworks/Elman.h"
#include "FastWanderer/include/FastWandererController.h"
#include "FastWanderer/include/FastWandererSharedData.h"

enum
{
    MLP_ID = 0,
    PERCEPTRON_ID = 1,
    ELMAN_ID = 2
};

FastWandererController::FastWandererController(RobotWorldModel *wm)
{
    m_wm = wm;
    std::vector<unsigned int> nbNeuronsPerHiddenLayers = getNbNeuronsPerHiddenLayers();
    unsigned int nbInputs = getNbInputs();
    unsigned int nbOutputs = getNbOutputs();

    switch (FastWandererSharedData::controllerType)
    {
        case MLP_ID:
            m_nn = new MLP(m_genome, nbInputs, nbOutputs, nbNeuronsPerHiddenLayers, true);
            break;
        case PERCEPTRON_ID:
            m_nn = new Perceptron(m_genome, nbInputs, nbOutputs);
            break;
        case ELMAN_ID:
            m_nn = new Elman(m_genome, nbInputs, nbOutputs, nbNeuronsPerHiddenLayers, true);
            break;
        default:
            std::cerr << "Invalid controller Type in " << __FILE__ << ":" << __LINE__ << ", got "
                      << FastWandererSharedData::controllerType << "\n";
            exit(-1);
    }
    m_genome.resize(m_nn->getRequiredNumberOfWeights(), 0);
    m_nn->setWeights(m_genome);
    resetFitness();
}

void FastWandererController::reset()
{
    if (FastWandererSharedData::controllerType == ELMAN_ID)
        dynamic_cast<Elman *>(m_nn)->initLastOutputs();
}

void FastWandererController::step()
{
    if (not m_wm->isAlive())
        return;

    std::vector<double> inputs = getInputs();

    m_nn->setInputs(inputs);
    m_nn->step();
    std::vector<double> outputs = m_nn->readOut();


    m_wm->_desiredTranslationalValue = (outputs[0] + 1.0) / 2.0 * gMaxTranslationalSpeed;
    m_wm->_desiredRotationalVelocity = outputs[1] * gMaxRotationalSpeed;
}

std::vector<unsigned int> FastWandererController::getNbNeuronsPerHiddenLayers() const
{
    auto nbHiddenLayers = static_cast<unsigned int>(FastWandererSharedData::nbHiddenLayers);
    std::vector<unsigned int> neuronsPerHiddenLayer(nbHiddenLayers);
    for (auto &nbNeuro : neuronsPerHiddenLayer)
    {
        nbNeuro = static_cast<unsigned int>(FastWandererSharedData::nbNeuronsPerHiddenLayer);
    }
    return neuronsPerHiddenLayer;
}


FastWandererController::~FastWandererController()
{
    delete m_nn;
}

std::vector<double> FastWandererController::getInputs()
{
    const int WALL_ID = 0;
    std::vector<double> inputs;
    inputs.reserve(m_nn->getNbInputs());

    /*
     * Camera inputs
     */
    for (int i = 0; i < m_wm->_cameraSensorsNb; i++)
    {
        inputs.emplace_back(m_wm->getDistanceValueFromCameraSensor(i) / m_wm->getCameraSensorMaximumDistanceValue(i));
    }

    return inputs;
}

void FastWandererController::loadNewGenome(const std::vector<double> &newGenome)
{
    m_genome = newGenome;
    m_nn->setWeights(m_genome);
    if (FastWandererSharedData::controllerType == ELMAN_ID)
        dynamic_cast<Elman *>(m_nn)->initLastOutputs();
}

unsigned int FastWandererController::getNbInputs() const
{
    return static_cast<unsigned int>(
            m_wm->_cameraSensorsNb
    );
}


double FastWandererController::getFitness() const
{
    return m_wm->_fitnessValue;
}


void FastWandererController::resetFitness()
{
    updateFitness(0);
}

void FastWandererController::updateFitness(double newFitness)
{
    if (newFitness < 0)
    {
        updateFitness(0);
        return;
    }
    m_wm->_fitnessValue = newFitness;
}

void FastWandererController::increaseFitness(double delta)
{
    updateFitness(m_wm->_fitnessValue + delta);
}

std::string FastWandererController::inspect(std::string prefix)
{
    std::stringstream out;
    double speed = (_wm->_actualTranslationalValue / gMaxTranslationalSpeed);
    double rotspeed = (_wm->_actualRotationalVelocity / gMaxRotationalSpeed);
    out << prefix << "speed: " << speed << "\n";
    out << prefix << "rotspeed: " << rotspeed << "\n";
    out << prefix << "Actual fitness: " << getFitness() << "\n";
    return out.str();
}

const std::vector<double> &FastWandererController::getGenome() const
{
    return m_genome;
}

unsigned int FastWandererController::getNbOutputs() const
{
    return 2    // Motor commands
        // Cooperation value
            ;
}


