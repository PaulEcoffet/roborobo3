//
// Created by paul on 27/10/17.
//

#include "neuralnetworks/Perceptron.h"
#include "Utilities/Misc.h"
#include "WorldModels/RobotWorldModel.h"
#include "Agents/Agent.h"
#include "RoboroboMain/main.h"
#include <set>
#include <PartnerChoice/include/PartnerChoiceWorldModel.h>
#include "neuralnetworks/Elman.h"
#include "PartnerChoice/include/PartnerChoiceController.h"
#include "PartnerChoice/include/PartnerChoiceSharedData.h"

enum
{
    MLP_ID = 0,
    PERCEPTRON_ID = 1,
    ELMAN_ID = 2
};

PartnerChoiceController::PartnerChoiceController(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<PartnerChoiceWorldModel *>(wm);
    std::vector<unsigned int> nbNeuronsPerHiddenLayers = getNbNeuronsPerHiddenLayers();
    unsigned int nbInputs = getNbInputs();
    unsigned int nbOutputs = getNbOutputs();

    switch (PartnerChoiceSharedData::controllerType)
    {
        case MLP_ID:
            m_nn = new MLP(m_weights, nbInputs, nbOutputs, nbNeuronsPerHiddenLayers, true);
            break;
        case PERCEPTRON_ID:
            m_nn = new Perceptron(m_weights, nbInputs, nbOutputs);
            break;
        case ELMAN_ID:
            m_nn = new Elman(m_weights, nbInputs, nbOutputs, nbNeuronsPerHiddenLayers, true);
            break;
        default:
            std::cerr << "Invalid controller Type in " << __FILE__ << ":" << __LINE__ << ", got "
                      << PartnerChoiceSharedData::controllerType << "\n";
            exit(-1);
    }
    m_weights.resize(m_nn->getRequiredNumberOfWeights(), 0);
    m_nn->setWeights(m_weights);
    resetFitness();
}

void PartnerChoiceController::reset()
{
    if (PartnerChoiceSharedData::controllerType == ELMAN_ID)
        dynamic_cast<Elman *>(m_nn)->initLastOutputs();
}

void PartnerChoiceController::step()
{
    if (not m_wm->isAlive())
        return;

    std::vector<double> inputs = getInputs();

    m_nn->setInputs(inputs);
    m_nn->step();
    std::vector<double> outputs = m_nn->readOut();


    m_wm->_desiredTranslationalValue = outputs[0] * gMaxTranslationalSpeed;
    m_wm->_desiredRotationalVelocity = outputs[1] * gMaxRotationalSpeed;


    m_wm->_cooperationLevel = outputs[2] + 1; // Range between [0; 2]
}

std::vector<unsigned int> PartnerChoiceController::getNbNeuronsPerHiddenLayers() const
{
    auto nbHiddenLayers = static_cast<unsigned int>(PartnerChoiceSharedData::nbHiddenLayers);
    std::vector<unsigned int> neuronsPerHiddenLayer(nbHiddenLayers);
    for (auto &nbNeuro : neuronsPerHiddenLayer)
    {
        nbNeuro = static_cast<unsigned int>(PartnerChoiceSharedData::nbNeuronsPerHiddenLayer);
    }
    return neuronsPerHiddenLayer;
}


PartnerChoiceController::~PartnerChoiceController()
{
    delete m_nn;
}

std::vector<double> PartnerChoiceController::getInputs()
{
    const int WALL_ID = 0;
    std::vector<double> inputs;
    inputs.reserve(m_nn->getNbInputs());

    /*
     * Camera inputs
     */
    for (int i = 0; i < m_wm->_cameraSensorsNb; i++)
    {
        bool isOpportunity = false;
        double seenCoop = 0;
        auto entityId = static_cast<int>(m_wm->getObjectIdFromCameraSensor(i));

        if (entityId >= gPhysicalObjectIndexStartOffset &&
            entityId < gPhysicalObjectIndexStartOffset + gNbOfPhysicalObjects) // is an Object
        {
            auto *opportunity = dynamic_cast<PartnerChoiceOpportunity *>(gPhysicalObjects[entityId -
                                                                                          gPhysicalObjectIndexStartOffset]);
            isOpportunity = true;
            if (PartnerChoiceSharedData::seeCoopFromDist)
                seenCoop = opportunity->getCoop();
            else
                seenCoop = 0;
        }
        inputs.emplace_back(m_wm->getDistanceValueFromCameraSensor(i) / m_wm->getCameraSensorMaximumDistanceValue(i));
        inputs.emplace_back(static_cast<double> (Agent::isInstanceOf(entityId)));
        inputs.emplace_back(static_cast<double> (entityId == WALL_ID));
        inputs.emplace_back(static_cast<double> (isOpportunity));
        inputs.emplace_back(seenCoop);
    }

    /*
     * Opportunity inputs
     */
    inputs.emplace_back(static_cast<double>(m_wm->onOpportunity));
    inputs.emplace_back(m_wm->meanLastTotalInvest());
    inputs.emplace_back(m_wm->meanLastOwnInvest());

    return inputs;
}

void PartnerChoiceController::loadNewGenome(const std::vector<double> &newGenome)
{
    m_weights = newGenome;
    m_nn->setWeights(m_weights);
    if (PartnerChoiceSharedData::controllerType == ELMAN_ID)
        dynamic_cast<Elman *>(m_nn)->initLastOutputs();
}

unsigned int PartnerChoiceController::getNbInputs() const
{
    return static_cast<unsigned int>(
            m_wm->_cameraSensorsNb * 5 // dist + isWall + isRobot + isObj + nbRob
            + 1 // onOpportunity
            + 1 // avgTotalEffort
            + 1 // avgEffort
    );
}


double PartnerChoiceController::getFitness() const
{
    return m_wm->_fitnessValue;
}


void PartnerChoiceController::resetFitness()
{
    updateFitness(0);
}

void PartnerChoiceController::updateFitness(double newFitness)
{
    if (newFitness < 0)
    {
        updateFitness(0);
        return;
    }
    m_wm->_fitnessValue = newFitness;
}

void PartnerChoiceController::increaseFitness(double delta)
{
    updateFitness(m_wm->_fitnessValue + delta);
}

std::string PartnerChoiceController::inspect(std::string prefix)
{
    std::stringstream out;

    std::set<int> seen;
    for (int i = 0; i < m_wm->_cameraSensorsNb; i++)
    {
        seen.insert((int) m_wm->getObjectIdFromCameraSensor(i));
    }

    out << "Seen objects:\n";
    for (int entityId : seen)
    {
        if (entityId == 0)
        {
            out << "\tA wall\n";
        }
        else if (Agent::isInstanceOf(entityId))
        {
            out << "\tAnother agent\n";
        }
        else if (entityId >= gPhysicalObjectIndexStartOffset)
        {
            out << "\tA cooperation opportunity ";
            auto coop = dynamic_cast<PartnerChoiceOpportunity *>(gPhysicalObjects[entityId -
                                                                                  gPhysicalObjectIndexStartOffset]);
            out << "with " << coop->getNbNearbyRobots() << " robots nearby.\n ";
        }
    }
    if (m_wm->onOpportunity)
    {
        out << "On opportunity\n";
        out << "\tLast own invest: ";
        for (auto ownInvest : m_wm->lastOwnInvest)
        {
            out << ownInvest << " ";
        }
        out << "(" << m_wm->meanLastOwnInvest() << ")";
        out << "\n";
        out << "\tLast total invest: ";
        for (auto totInvest : m_wm->lastTotalInvest)
        {
            out << totInvest << " ";
        }
        out << "(" << m_wm->meanLastTotalInvest() << ")";
        out << "\n";

    }
    out << "Actual fitness: " << getFitness() << "\n";
    return out.str();
}

unsigned long PartnerChoiceController::getGenomeSize() const
{
    return m_weights.size();
}

unsigned int PartnerChoiceController::getNbOutputs() const
{
    return 2    // Motor commands
           + 1  // Cooperation value
            ;
}


